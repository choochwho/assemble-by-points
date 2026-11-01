"""Merge fitted parts into one printable solid.

Pipeline (all on copies; the input objects are never modified):

1. Prepare each part: world-space copy, collapse exact duplicate vertices,
   fill holes in the scan (a boolean only gives a closed result from closed
   input). Away from the seam, nothing in a part is ever merged or thinned.
2. Close gaps between the fitted pair: the mating faces are first subdivided
   where they're coarser than the target spacing, so even low-poly parts have
   vertices to move (dense scans are left as they are). Every
   vertex on a mating surface casts a ray along the axis of assembly to the
   other part; where the gap is small both sides move halfway across it (plus
   a little, so they overlap). The seam's rim edges bend to meet instead of
   leaving a slit. Wider gaps fade out and are left alone.
3. Boolean-union everything.
4. Delete internal voids and slivers, cut out and re-fill any remaining
   non-manifold spots, so what's left is one closed outer shell.
5. Even out the seam band: split its edges longer than twice the merge
   distance, collapse those shorter than it, so the seam ends up about as
   dense as the target spacing; nothing outside the band is thinned.
6. Smooth the seam: blend the two parts' walls across the band. The areas
   where the parts touched are inside the solid by now, and everything outside
   the band is left exactly as it was.

Every step that moves vertices first splits the long edges running away from
them, so a move only tilts short faces next to it, never a large face
reaching far from the seam (which would shift detail elsewhere).
"""

import bmesh
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree

FACING = 0.2          # |normal . axis| below this: a side wall, not a mating surface
GAP_FADE = 0.5        # gaps up to (1 + GAP_FADE) x the limit are partly closed
REPAIR_PASSES = 4
DENSIFY_ROUNDS = 6    # each round halves the long edges
SMOOTH_ITERATIONS = 30      # Taubin passes when there's no axis to blend along
RELAX_ITERATIONS = 4        # light passes after blending, to even out the triangles
WALL_LIMIT = 0.8            # |normal . axis| above this: a mating face, not an outer wall


def boolean_solvers():
    """Solvers the running Blender supports (MANIFOLD needs 4.5+)."""
    prop = bpy.types.BooleanModifier.bl_rna.properties["solver"]
    return [item.identifier for item in prop.enum_items]


# --------------------------------------------------------------------------
# Reports
# --------------------------------------------------------------------------

def _bm_report(bm):
    report = {
        "non_manifold_edges": sum(1 for e in bm.edges if not e.is_manifold),
        "boundary_edges": sum(1 for e in bm.edges if e.is_boundary),
        "wire_edges": sum(1 for e in bm.edges if e.is_wire),
        "non_manifold_verts": sum(1 for v in bm.verts if not v.is_manifold),
        "loose_verts": sum(1 for v in bm.verts if not v.link_edges),
    }
    report["is_manifold"] = not any(report.values())
    report["shells"] = len(_shells(bm))
    return report


def manifold_report(mesh):
    """Count problems that stop a mesh from being a closed 2-manifold."""
    bm = bmesh.new()
    bm.from_mesh(mesh)
    report = _bm_report(bm)
    bm.free()
    return report


def describe(report):
    if report["is_manifold"]:
        return "manifold"
    parts = [f"{report[k]} {k.replace('_', ' ')}" for k in
             ("non_manifold_edges", "boundary_edges", "wire_edges",
              "non_manifold_verts", "loose_verts") if report.get(k)]
    return "NOT manifold: " + ", ".join(parts)


# --------------------------------------------------------------------------
# Mesh helpers
# --------------------------------------------------------------------------

def _shells(bm):
    """Face-connected pieces, as lists of faces."""
    seen, shells = set(), []
    for start in bm.faces:
        if start in seen:
            continue
        shell, stack = [], [start]
        seen.add(start)
        while stack:
            f = stack.pop()
            shell.append(f)
            for e in f.edges:
                for g in e.link_faces:
                    if g not in seen:
                        seen.add(g)
                        stack.append(g)
        shells.append(shell)
    return shells


def _signed_volume(faces):
    vol = 0.0
    for f in faces:
        co = [v.co for v in f.verts]
        for i in range(1, len(co) - 1):
            vol += co[0].dot(co[i].cross(co[i + 1]))
    return vol / 6.0


def _remove_loose(bm):
    wire = [e for e in bm.edges if not e.link_faces]
    if wire:
        bmesh.ops.delete(bm, geom=wire, context="EDGES")
    loose = [v for v in bm.verts if not v.link_edges]
    if loose:
        bmesh.ops.delete(bm, geom=loose, context="VERTS")


def _fill_holes(bm):
    """Cap every open boundary loop. Returns the number of faces added."""
    boundary = [e for e in bm.edges if e.is_boundary]
    if not boundary:
        return 0
    faces = bmesh.ops.holes_fill(bm, edges=boundary, sides=0)["faces"]
    if faces:
        bmesh.ops.triangulate(bm, faces=faces)
    return len(faces)


def _remove_voids(bm):
    """Delete inside-out shells (cavities enclosed inside the solid) and
    zero-volume slivers. Must run before normals are recalculated, which
    would turn cavities outward. Returns the number removed."""
    shells = [(shell, _signed_volume(shell)) for shell in _shells(bm)]
    if not shells:
        return 0
    tiny = 1e-6 * max(vol for _s, vol in shells)
    voids = [shell for shell, vol in shells if vol < tiny]
    for shell in voids:
        bmesh.ops.delete(bm, geom=shell, context="FACES")
    return len(voids)


def _repair(bm):
    """Fix what keeps the mesh from being a closed manifold: fill open holes,
    and cut out and re-fill faces around tangled spots (edges with more than
    two faces, vertices where two fans touch). Never deletes around a hole it
    couldn't fill, so a repair can't keep growing into the rest of the part.
    Returns the number of faces cut out."""
    cut = 0
    for _ in range(REPAIR_PASSES):
        _fill_holes(bm)
        bad = _ordered(f for e in bm.edges if len(e.link_faces) > 2 for f in e.link_faces)
        bad += [f for f in _ordered(f for v in bm.verts
                                    if not v.is_manifold and not v.is_boundary
                                    for f in v.link_faces) if f not in set(bad)]
        if not bad:
            break
        cut += len(bad)
        bmesh.ops.delete(bm, geom=bad, context="FACES")
        _remove_loose(bm)
        _fill_holes(bm)
    return cut


def _finish(bm, report, passes=3):
    """Final cleanup until the mesh is one clean closed shell (or nothing more
    can be done): stray edges and vertices, voids and slivers, then repairs.
    Returns the number of faces repaired."""
    cut = 0
    for _ in range(passes):
        _remove_loose(bm)
        report["voids_removed"] += _remove_voids(bm)
        _remove_loose(bm)
        done = _repair(bm)
        cut += done
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        if not done and _bm_report(bm)["is_manifold"]:
            break
    return cut


def world_bmesh(obj, depsgraph):
    """BMesh of ``obj`` with modifiers and its world transform applied."""
    eval_obj = obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()
    bm = bmesh.new()
    try:
        bm.from_mesh(mesh)
    finally:
        eval_obj.to_mesh_clear()
    bm.transform(obj.matrix_world)
    if obj.matrix_world.is_negative:
        bmesh.ops.reverse_faces(bm, faces=bm.faces)
    return bm


def prepare(bm, duplicate_distance):
    """Make a part ready for a boolean. Only exact duplicates (split vertices
    in the scan data) are collapsed, so no detail is lost. Returns the number
    of hole faces added."""
    if duplicate_distance > 0:
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=duplicate_distance)
    _remove_loose(bm)
    filled = _fill_holes(bm)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.normal_update()
    return filled


# --------------------------------------------------------------------------
# Gap closing
# --------------------------------------------------------------------------

def _gap_moves(bm, other, toward, max_gap, overlap):
    """How far each mating-surface vertex of ``bm`` should move ``toward`` the
    ``other`` part (a BVHTree) to meet it halfway across a small gap.
    Returns {vertex: move vector}."""
    moves = {}
    reach = max_gap * (1 + GAP_FADE)
    for v in bm.verts:
        facing = v.normal.dot(toward)
        if facing < FACING:
            continue
        loc, normal, _i, gap = other.ray_cast(v.co, toward, reach)
        if loc is None:
            # Vertices right on the rim fire their ray down the outside edge
            # of the other part and miss; use its nearest point instead, if
            # it lies ahead along the axis rather than off to the side.
            loc, normal, _i, _d = other.find_nearest(v.co, reach)
            if loc is None:
                continue
            gap = (loc - v.co).dot(toward)
            if gap <= 0 or (loc - v.co - toward * gap).length > gap + 1e-9:
                continue
        # The other surface must face back at us; if its normal points along
        # the ray, we're already inside the other part (overlapping).
        if normal.dot(toward) > -FACING:
            continue
        fade = 1.0 if gap <= max_gap else 1.0 - (gap - max_gap) / (GAP_FADE * max_gap)
        weight = min(1.0, (facing - FACING) / 0.3) * max(fade, 0.0)
        if weight > 0:
            moves[v] = toward * ((gap + overlap) / 2 * weight)
    return _fill_skipped(moves, toward)


def _fill_skipped(moves, toward, passes=2):
    """Give vertices skipped by the ray test (noisy normals, rim corners) the
    average move of their neighbours when most of those neighbours moved, so
    the closed seam has no notches. Vertices facing away are left alone."""
    for _ in range(passes):
        added = {}
        candidates = _ordered(e.other_vert(v) for v in moves for e in v.link_edges
                              if e.other_vert(v) not in moves)
        for v in candidates:
            if v.normal.dot(toward) < -FACING:
                continue
            near = [e.other_vert(v) for e in v.link_edges]
            moved = [moves[n] for n in near if n in moves]
            if len(moved) * 2 > len(near):
                added[v] = sum(moved, Vector()) / len(moved)
        if not added:
            break
        moves.update(added)
    return moves


def _typical_edge(*bms, samples=20000):
    """Median edge length, from an even sample of each mesh's edges."""
    lengths = []
    for bm in bms:
        bm.edges.ensure_lookup_table()
        edges = bm.edges                # BMesh sequences can't be sliced with a step
        step = max(1, len(edges) // samples)
        lengths += [edges[i].calc_length() for i in range(0, len(edges), step)]
    lengths.sort()
    return lengths[len(lengths) // 2] if lengths else 0.0


def close_gaps(bm_target, bm_moving, axis, max_gap, min_overlap, spacing):
    """Move both mating surfaces halfway across gaps up to ``max_gap`` wide,
    measured along ``axis`` (pointing from target to moving), and a little
    further so they overlap. Returns the number of vertices moved.

    The overlap is at least half the typical edge length: gaps are only
    measured at vertices, and in between the other scan's surface noise must
    not poke out again as hairline slits along the outer seam.
    """
    axis = Vector(axis).normalized()
    overlap = min(max(min_overlap, 0.5 * _typical_edge(bm_target, bm_moving)), max_gap)
    bvh_t = BVHTree.FromBMesh(bm_target)
    bvh_m = BVHTree.FromBMesh(bm_moving)
    # Keep the change local: split long edges from moved to unmoved vertices
    # (e.g. a tall side wall under a rim vertex) so a move only tilts short
    # faces next to it, not a whole face reaching far from the seam. Splitting
    # invalidates BMVert references but never moves a vertex, so the moves are
    # remembered by position; the new vertices stay put.
    moves = {}
    for bm, other, toward in ((bm_target, bvh_m, axis), (bm_moving, bvh_t, -axis)):
        by_pos = {v.co.to_tuple(): d
                  for v, d in _gap_moves(bm, other, toward, max_gap, overlap).items()}
        for _ in range(DENSIFY_ROUNDS):
            moved = {v for v in bm.verts if v.co.to_tuple() in by_pos}
            long_edges = _ordered(e for v in moved for e in v.link_edges
                                  if (e.verts[0] in moved) != (e.verts[1] in moved)
                                  and e.calc_length() > 2 * spacing)
            if not long_edges:
                break
            _split_edges(bm, long_edges)
        moves.update((v, by_pos[v.co.to_tuple()]) for v in bm.verts
                     if v.co.to_tuple() in by_pos)
    for v, delta in moves.items():  # applied after measuring, so both sides see
        v.co += delta               # the original, undeformed surfaces
    bm_target.normal_update()
    bm_moving.normal_update()
    return len(moves)


def _split_edges(bm, edges):
    """Split edges in half with the faces around them. Like any bmesh
    operator this can invalidate vertex references; callers never keep any."""
    bmesh.ops.subdivide_edges(bm, edges=[e for e in edges if e.is_valid], cuts=1,
                              use_grid_fill=True)


def _densify(bm, pick_edges, spacing, rounds=DENSIFY_ROUNDS):
    """Subdivide the edges ``pick_edges(bm)`` returns that are longer than
    ``spacing``, until none are left or ``rounds`` run out. Adds vertices
    without changing the shape. Returns the number of edges split."""
    split = 0
    for _ in range(rounds):
        edges = _ordered(e for e in pick_edges(bm) if e.calc_length() > spacing)
        if not edges:
            break
        split += len(edges)
        _split_edges(bm, edges)
    bm.normal_update()
    return split


def _mating_edges(other, toward, reach):
    """Edge picker: edges of faces facing ``toward`` the other part within reach."""
    def pick(bm):
        out = {}
        for f in bm.faces:
            if f.normal.dot(toward) < FACING:
                continue
            c = f.calc_center_median()
            radius = max((v.co - c).length for v in f.verts)
            if other.find_nearest(c, reach + radius)[0] is not None:
                out.update(dict.fromkeys(f.edges))
        return list(out)
    return pick


def densify_mating(bm_target, bm_moving, axis, max_gap, spacing):
    """Subdivide coarse faces on both mating surfaces so gap closing has
    vertices to move. Returns the number of edges split."""
    axis = Vector(axis).normalized()
    reach = max_gap * (1 + GAP_FADE)
    bvh_t = BVHTree.FromBMesh(bm_target)
    bvh_m = BVHTree.FromBMesh(bm_moving)
    split = _densify(bm_target, _mating_edges(bvh_m, axis, reach), 2 * spacing)
    split += _densify(bm_moving, _mating_edges(bvh_t, -axis, reach), 2 * spacing)
    return split


def _sources(verts, parts, tol):
    """For each vertex, the parts whose surface lies within ``tol`` of it."""
    return {v: frozenset(i for i, bvh in enumerate(parts)
                         if bvh.find_nearest(v.co, tol)[0] is not None) for v in verts}


def _ordered(items):
    """De-duplicate, keeping first-seen order (sets of BMesh elements iterate
    in memory-address order, which would make results vary between runs)."""
    return list(dict.fromkeys(items))


def _kdtree(points):
    kd = KDTree(len(points))
    for i, co in enumerate(points):
        kd.insert(co, i)
    kd.balance()
    return kd


class SeamBand:
    """The band of surface within ``width`` of the seam: where the parts'
    outer surfaces meet (vertices lying on two parts' surfaces). The areas
    where the parts touched are inside the solid by now, so they're never
    part of it."""

    def __init__(self, bm, parts, width, on_surface, axis=None, walls=None):
        """``walls`` = (target, moving) BVHs of each part's outer walls, and
        ``axis`` points from target to moving; without them the band is only
        Taubin-smoothed instead of blended."""
        self.width = width
        self.axis = Vector(axis).normalized() if axis is not None else None
        self.walls = walls if walls and all(walls) else None
        self.seam = [v.co.copy() for v, src in _sources(bm.verts, parts, on_surface).items()
                     if len(src) >= 2]
        self.kd = _kdtree(self.seam) if self.seam else None

    def dist(self, co):
        return self.kd.find(co)[2]

    def near_verts(self, bm, reach):
        """Vertices within ``reach`` of the seam, found from the seam's side
        (a KD-tree over the mesh, queried at each seam point), so the rest of
        a large mesh is never visited."""
        bm.verts.ensure_lookup_table()
        kd = _kdtree([v.co for v in bm.verts])
        found = set()
        for co in self.seam:
            found.update(i for _co, i, _d in kd.find_range(co, reach))
        return [bm.verts[i] for i in sorted(found)]

    def even_out(self, bm, spacing, native, degenerate):
        """Bring the seam area close to the parts' own density: split edges
        reaching into the band longer than twice ``spacing`` (so the band has
        vertices to smooth, and faces running out of it are split near it,
        so smoothing can't tilt a face far away), then collapse band edges
        shorter than half of ``spacing`` or of the parts' ``native`` edge
        length, whichever is smaller: the boolean's slivers along the seam,
        without thinning the seam below the density around it. Nothing
        outside the band is touched. Returns (edges split, vertices welded)."""
        reach = max(self.width, 2 * spacing)

        def reaching(b):
            return _ordered(e for v in self.near_verts(b, reach) for e in v.link_edges)

        split = _densify(bm, reaching, 2 * spacing)
        # Collapse short *edges* (only connected vertices merge; welding by
        # distance could join the two sides of a thin sliver and tear the mesh).
        limit = 0.5 * min(spacing, native)
        near = set(self.near_verts(bm, reach))
        short = [e for e in _ordered(e for v in near for e in v.link_edges)
                 if e.verts[0] in near and e.verts[1] in near and e.calc_length() < limit]
        before = len(bm.verts)
        if short:
            bmesh.ops.dissolve_degenerate(bm, edges=short, dist=limit)
        return split, before - len(bm.verts)

    def smooth(self, bm):
        """Blend the two parts' walls across the band (or just Taubin-smooth it
        when there's no axis). Returns the number of vertices moved."""
        bm.normal_update()
        moved = self._blend(bm) if self.axis is not None and self.walls else 0
        weights = self._weights(bm)
        self._taubin(weights, SMOOTH_ITERATIONS if not moved else RELAX_ITERATIONS)
        bm.normal_update()
        return max(moved, len(weights) if not moved else moved)

    def _weights(self, bm):
        """Smoothing strength per band vertex: 1 at the seam, 0 at the edge."""
        weights = {}
        for v in self.near_verts(bm, self.width):
            d = self.dist(v.co)
            if d >= self.width or not v.link_edges:
                continue
            t = d / self.width
            weights[v] = 1 - t * t * (3 - 2 * t)        # smoothstep falloff
        return weights

    def _blend(self, bm):
        """Move each band vertex onto a blend of the two parts' walls.

        Each vertex is projected onto (the plane of) the nearest outer-wall
        face of the target and of the moving part, and placed between those
        projections by how far along the axis it sits across the band: all
        target at one edge, all moving part at the other, smoothly between.
        At the band's edges that's exactly the vertex's own surface, so
        nothing outside it changes. Where the walls are more than a band
        width apart (a real overhang, not a seam step) it's left alone.
        """
        w, a = self.width, self.axis
        wall_t, wall_m = self.walls
        targets = {}
        for v in self.near_verts(bm, w):
            seam_pt, _i, d = self.kd.find(v.co)
            if d >= w:
                continue
            t = min(1.0, max(0.0, ((v.co - seam_pt).dot(a) + w) / (2 * w)))
            s = t * t * (3 - 2 * t)
            lt, nt, _i, _d = wall_t.find_nearest(v.co, 2 * w)
            lm, nm, _i, _d = wall_m.find_nearest(v.co, 2 * w)
            if lt is None or lm is None:
                continue
            on_t = v.co - nt * (v.co - lt).dot(nt)
            on_m = v.co - nm * (v.co - lm).dot(nm)
            if (on_t - on_m).length > w:
                continue
            targets[v] = on_t.lerp(on_m, s)
        for v, co in targets.items():
            v.co = co
        return len(targets)

    @staticmethod
    def _taubin(weights, iterations):
        # Alternating shrink/inflate steps smooth without shrinking the part
        # the way plain Laplacian smoothing would.
        for _ in range(iterations):
            for lam in (0.5, -0.53):
                new = {}
                for v, wt in weights.items():
                    ring = [e.other_vert(v).co for e in v.link_edges]
                    new[v] = v.co + (sum(ring, Vector()) / len(ring) - v.co) * (lam * wt)
                for v, co in new.items():
                    v.co = co


def wall_bvh(bm, axis):
    """BVH of a part's outer walls: every face except those nearly
    perpendicular to the axis (its mating surfaces)."""
    axis = Vector(axis).normalized()
    bm.verts.index_update()
    bm.normal_update()
    polys = [[v.index for v in f.verts] for f in bm.faces
             if abs(f.normal.dot(axis)) < WALL_LIMIT]
    if not polys:
        return None
    return BVHTree.FromPolygons([v.co.copy() for v in bm.verts], polys)


# --------------------------------------------------------------------------
# Merge
# --------------------------------------------------------------------------

def _boolean_union(meshes, solver, context, collection):
    """Union world-space meshes with the boolean modifier. Returns a new mesh."""
    objs = []
    try:
        for i, mesh in enumerate(meshes):
            obj = bpy.data.objects.new(f"_assembly_tmp_{i}", mesh)
            collection.objects.link(obj)
            objs.append(obj)
        base = objs[0]
        for obj in objs[1:]:
            mod = base.modifiers.new(f"Union {obj.name}", "BOOLEAN")
            mod.operation, mod.solver, mod.object = "UNION", solver, obj
        context.view_layer.update()
        depsgraph = context.evaluated_depsgraph_get()
        return bpy.data.meshes.new_from_object(base.evaluated_get(depsgraph),
                                               depsgraph=depsgraph)
    finally:
        for obj in objs:
            mesh = obj.data
            bpy.data.objects.remove(obj)
            bpy.data.meshes.remove(mesh)


def merge(objects, context, spacing, max_gap, seam_width=0.0, gap_pair=None,
          axis=None, name=None):
    """Merge ``objects`` into a new object and return (object, report).

    ``spacing`` is the merge distance: the seam area ends up with vertices
    about that far apart (never closer), and no part is subdivided finer.
    ``gap_pair`` is (target, moving) among ``objects`` whose gaps are closed
    along ``axis`` (world space, pointing from target to moving).
    """
    if len(objects) < 2:
        raise ValueError("Need at least two mesh objects to merge")
    depsgraph = context.evaluated_depsgraph_get()
    name = name or "_".join(o.name for o in objects[:2]) + "_merged"
    size = max(o.dimensions.length for o in objects)
    # "This vertex lies on that part's surface". Loose enough for the boolean
    # solver's own vertex snapping; the parts only meet at the seam (their
    # contact is inside the solid), so this can't mark anything else as seam.
    on_surface = 2e-4 * size
    report = {"holes_filled": 0, "gap_vertices_moved": 0, "edges_split": 0}

    bms = {o: world_bmesh(o, depsgraph) for o in objects}
    try:
        for bm in bms.values():
            report["holes_filled"] += prepare(bm, 1e-7 * size)
        native = _typical_edge(*bms.values())      # the parts' own vertex spacing
        if gap_pair and axis is not None and max_gap > 0:
            bm_t, bm_m = bms[gap_pair[0]], bms[gap_pair[1]]
            # Mating faces only need vertices about as far apart as the gaps
            # they bend across; dense scans aren't touched.
            step = max(spacing, max_gap)
            report["edges_split"] = densify_mating(bm_t, bm_m, axis, max_gap, step)
            overlap = max(2 * spacing, 1e-4 * size)
            report["gap_vertices_moved"] = close_gaps(bm_t, bm_m, axis, max_gap, overlap, step)
        parts = [BVHTree.FromBMesh(bm) for bm in bms.values()]
        walls = None
        if gap_pair and axis is not None:
            walls = (wall_bvh(bms[gap_pair[0]], axis), wall_bvh(bms[gap_pair[1]], axis))
        closed = all(_bm_report(bm)["is_manifold"] for bm in bms.values())
        solver = "MANIFOLD" if closed and "MANIFOLD" in boolean_solvers() else "EXACT"
        meshes = []
        for obj, bm in bms.items():
            mesh = bpy.data.meshes.new(f"_assembly_tmp_{obj.name}")
            bm.to_mesh(mesh)
            meshes.append(mesh)
    finally:
        for bm in bms.values():
            bm.free()

    collection = objects[0].users_collection[0] if objects[0].users_collection \
        else context.scene.collection
    merged = _boolean_union(meshes, solver, context, collection)
    merged.name = name
    report["solver"] = solver

    bm = bmesh.new()
    bm.from_mesh(merged)
    degenerate = 1e-6 * size
    bmesh.ops.dissolve_degenerate(bm, edges=list(bm.edges), dist=degenerate)
    report["voids_removed"] = _remove_voids(bm)
    _remove_loose(bm)
    report["faces_repaired"] = _repair(bm)
    if report["faces_repaired"]:            # a repair can leave new slivers
        report["voids_removed"] += _remove_voids(bm)
        _remove_loose(bm)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    # Even out the seam only on cleaned-up geometry (subdividing raw boolean
    # output can crash Blender).
    band = SeamBand(bm, parts, seam_width, on_surface, axis, walls)
    report["seam_split"] = report["seam_collapsed"] = 0
    if band.kd is not None and spacing > 0:
        report["seam_split"], report["seam_collapsed"] = band.even_out(bm, spacing, native,
                                                                       degenerate)
        if report["seam_collapsed"]:
            cut = _repair(bm)
            if cut:
                report["faces_repaired"] += cut
                report["voids_removed"] += _remove_voids(bm)
                _remove_loose(bm)
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    report["seam_smoothed"] = band.smooth(bm) if band.kd is not None and seam_width > 0 else 0
    report["faces_repaired"] += _finish(bm, report)
    report.update(_bm_report(bm))
    bm.to_mesh(merged)
    bm.free()

    result = bpy.data.objects.new(name, merged)
    collection.objects.link(result)
    return result, report
