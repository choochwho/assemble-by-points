"""Headless tests. Run with:

    blender --background --factory-startup --python tests/run_tests.py
"""

import math
import os
import random
import sys
import time
import traceback

import bmesh
import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import assemble_by_points  # noqa: E402
from assemble_by_points import merge  # noqa: E402
from assemble_by_points.operators import add_pair  # noqa: E402


# ---------------------------------------------------------------- scene helpers

def set_origin(obj, location):
    """Move ``obj``'s origin to world ``location`` without moving its geometry
    (origins play no part in fitting; this just keeps the scenes realistic)."""
    local = obj.matrix_world.inverted() @ location
    obj.data.transform(Matrix.Translation(-local))
    obj.matrix_world = obj.matrix_world @ Matrix.Translation(local)


def reset():
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    s = bpy.context.scene.assemble_by_points
    for prop in ("tilt_limit", "spin_range", "merge_pct", "gap_pct", "seam_pct",
                 "fit_target", "fit_moving", "fit_axis"):
        s.property_unset(prop)
    s.pairs.clear()
    s.enabled = True


def select(*objs, active):
    for o in bpy.context.scene.objects:
        o.select_set(o in objs)
    bpy.context.view_layer.objects.active = active


def interface(x, y):
    """Mating surface: a broad wave, a groove along x and a round 0.5 peg."""
    wave = 0.6 * math.sin(0.5 * x + 1) * math.cos(0.4 * y) + 0.05 * x * y
    groove = -0.4 * math.exp(-((y + 2) / 0.35) ** 2)
    r = math.hypot(x - 2, y - 1.5)
    peg = 0.5 / (1 + math.exp(min((r - 0.5) / 0.06, 50)))
    return wave + groove + peg


def scan_piece(name, cap_z, res, noise, seed, half=5.0, surface=interface):
    """Solid block over [-half, half]^2 between ``surface`` and z = cap_z.

    Each piece gets its own tessellation and random surface noise, like two
    separate scans of parts that fit together in real life.
    """
    rng = random.Random(seed)
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=res, y_segments=res, size=half)
    for vert in bm.verts:
        vert.co.z = surface(vert.co.x, vert.co.y) + rng.gauss(0, noise)
    ret = bmesh.ops.extrude_face_region(bm, geom=list(bm.faces))
    for vert in (g for g in ret["geom"] if isinstance(g, bmesh.types.BMVert)):
        vert.co.z = cap_z
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    set_origin(obj, Vector((0, 0, cap_z)))       # origins at the far ends
    return obj


def make_pair(with_pairs=True):
    """Two scanned halves in their true, assembled pose, with three matching
    points clicked (a little inaccurately) on the mating surfaces."""
    bottom = scan_piece("Bottom", -3.0, 90, 0.02, seed=1)
    top = scan_piece("Top", 3.0, 77, 0.02, seed=2)
    if with_pairs:
        click_pairs(top, bottom, SOLID_SPOTS)
    return bottom, top


def hollow_out(obj, cap_z, wall=0.5, half=5.0):
    """Cut a cavity into a scan piece, open on the mating side, leaving walls
    ``wall`` thick: only the rims meet when the two halves are assembled."""
    inner = half - wall
    z0, z1 = (cap_z + wall, -cap_z) if cap_z < 0 else (-cap_z, cap_z - wall)
    mesh = bpy.data.meshes.new("cutter")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=Vector((2 * inner, 2 * inner, z1 - z0)), verts=bm.verts)
    bmesh.ops.translate(bm, vec=Vector((0, 0, (z0 + z1) / 2)), verts=bm.verts)
    bm.to_mesh(mesh)
    bm.free()
    cutter = bpy.data.objects.new("cutter", mesh)
    bpy.context.scene.collection.objects.link(cutter)
    mod = obj.modifiers.new("hollow", "BOOLEAN")
    mod.operation, mod.solver, mod.object = "DIFFERENCE", "EXACT", cutter
    cutter.matrix_world = obj.matrix_world.copy()       # both built in world space
    cutter.matrix_world = Matrix.Identity(4)
    dg = bpy.context.evaluated_depsgraph_get()
    new = bpy.data.meshes.new_from_object(obj.evaluated_get(dg), depsgraph=dg)
    obj.modifiers.clear()
    old, obj.data = obj.data, new
    new.transform(Matrix.Identity(4))
    bpy.data.meshes.remove(old)
    bpy.data.objects.remove(cutter)
    bpy.data.meshes.remove(mesh)


def make_hollow_pair(surface=interface):
    bottom = scan_piece("Bottom", -3.0, 90, 0.02, seed=1, surface=surface)
    top = scan_piece("Top", 3.0, 77, 0.02, seed=2, surface=surface)
    hollow_out(bottom, -3.0)
    hollow_out(top, 3.0)
    set_origin(bottom, Vector((-0.1, 0.1, -3.0)))
    set_origin(top, Vector((0.15, -0.1, 3.0)))
    return bottom, top


def click_pairs(top, bottom, spots, surface=interface, error=0.08, seed=5):
    """Matching pairs on the rims, clicked a little inaccurately on each side."""
    rng = random.Random(seed)
    for x, y in spots:
        true = Vector((x, y, surface(x, y)))
        jitter = lambda: Vector((rng.uniform(-error, error), rng.uniform(-error, error), 0))
        add_pair(bpy.context.scene, top, true + jitter(), bottom, true + jitter())


RIM_SPOTS = [(4.75, -1.0), (-4.75, 2.5), (1.5, 4.75), (-2.0, -4.75)]
SOLID_SPOTS = [(3.0, -3.0), (-3.5, 1.0), (1.0, 3.5)]


def about_origin(obj, R):
    o = obj.matrix_world.translation.copy()
    return Matrix.Translation(o) @ R.to_4x4() @ Matrix.Translation(-o)


def displace(obj, spin_deg, tilt_deg, offset=None, pull_back=None, lateral=(0, 0, 0)):
    """Spin about z and tilt about x (both about the origin), then move away:
    either by a raw ``offset`` or ``pull_back`` along the true join direction
    (z; how a user would lay parts out facing each other) plus a small
    ``lateral`` placement error."""
    R = Matrix.Rotation(math.radians(tilt_deg), 3, "X") @ Matrix.Rotation(
        math.radians(spin_deg), 3, "Z")
    if offset is None:
        offset = Vector((0, 0, pull_back)) + Vector(lateral)
    obj.matrix_world = Matrix.Translation(offset) @ about_origin(obj, R) @ obj.matrix_world


def pose_error(a, b):
    """Max displacement of the 10 x 10 x 6 block's corners between two poses."""
    corners = [Vector((x, y, z)) for x in (-5, 5) for y in (-5, 5) for z in (0, 6)]
    return max(((a @ c) - (b @ c)).length for c in corners)


def run_fit(target, moving, **settings):
    s = bpy.context.scene.assemble_by_points
    for k, v in settings.items():
        setattr(s, k, v)
    select(target, moving, active=target)
    t0 = time.perf_counter()
    assert bpy.ops.assemble_by_points.fit() == {"FINISHED"}
    return time.perf_counter() - t0


def volume(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.transform(obj.matrix_world)
    vol = bm.calc_volume(signed=True)
    bm.free()
    return vol


# ---------------------------------------------------------------- tests

def test_fit_from_far_away_default_settings():
    reset()
    bottom, top = make_pair()
    truth = top.matrix_world.copy()
    displace(top, 60, 3, pull_back=20, lateral=(0.3, -0.2, 0))
    secs = run_fit(bottom, top)
    err = pose_error(top.matrix_world, truth)
    print(f"  pose error {err:.3f} in {secs:.1f}s")
    assert err < 0.1, err


def test_full_spin_range():
    reset()
    bottom, top = make_pair()
    truth = top.matrix_world.copy()
    displace(top, -150, -2, pull_back=15, lateral=(-0.2, 0.25, 0))
    secs = run_fit(bottom, top, spin_range=math.radians(360))
    err = pose_error(top.matrix_world, truth)
    print(f"  pose error {err:.3f} in {secs:.1f}s")
    assert err < 0.1, err


def test_spin_range_is_respected():
    """A 150 degree spin is outside the default +-90, so the fit may not undo it."""
    reset()
    bottom, top = make_pair()
    displace(top, 150, 0, pull_back=20)
    before = top.matrix_world.to_3x3()
    run_fit(bottom, top)
    delta = top.matrix_world.to_3x3() @ before.inverted()
    angle = math.degrees(delta.to_quaternion().angle)
    print(f"  turned {angle:.1f} deg")
    assert angle <= 90 + 5 + 0.5, angle


def test_larger_tilt_needs_larger_limit():
    reset()
    bottom, top = make_pair()
    truth = top.matrix_world.copy()
    displace(top, 20, 12, pull_back=12, lateral=(0.2, 0.1, 0))
    secs = run_fit(bottom, top, tilt_limit=math.radians(15))
    err = pose_error(top.matrix_world, truth)
    print(f"  pose error {err:.3f} in {secs:.1f}s")
    assert err < 0.1, err


def test_placed_off_to_the_side_with_big_spin():
    """The moving part laid out well off to the side and spun 150 degrees: the
    matching points define the axis, so the default tilt limit is enough."""
    reset()
    bottom, top = make_pair()
    truth = top.matrix_world.copy()
    displace(top, -150, 0, offset=(2.5, -1.5, 25))
    secs = run_fit(bottom, top, spin_range=math.radians(360))
    err = pose_error(top.matrix_world, truth)
    print(f"  pose error {err:.3f} in {secs:.1f}s")
    assert err < 0.1, err


def merge_selected(target, moving):
    select(target, moving, active=target)
    t0 = time.perf_counter()
    assert bpy.ops.assemble_by_points.merge() == {"FINISHED"}
    result = bpy.context.view_layer.objects.active
    rep = merge.manifold_report(result.data)
    vol = volume(result)
    print(f"  merged in {time.perf_counter() - t0:.1f}s: {merge.describe(rep)}, "
          f"{rep['shells']} shell(s), {len(result.data.polygons)} faces, volume {vol:.2f}")
    return result, rep, vol


def seam_depth(result, z_at, samples=12, half=5.0):
    """How far rays fired inward at the seam height get past the outer wall.
    ~0 means the seam is closed; a slit lets rays in deeper."""
    from mathutils.bvhtree import BVHTree
    dg = bpy.context.evaluated_depsgraph_get()
    bvh = BVHTree.FromObject(result, dg)
    worst = 0.0
    for k in range(samples):
        y = -half * 0.8 + 1.6 * half * k / (samples - 1)
        for sign in (1, -1):
            start = Vector((sign * (half + 3), y, z_at(sign * half, y)))
            loc = bvh.ray_cast(start, Vector((-sign, 0, 0)))[0]
            depth = half - abs(loc.x) if loc is not None else half
            worst = max(worst, depth)
    return worst


def test_merge_after_fit():
    reset()
    bottom, top = make_pair()
    displace(top, 30, 2, pull_back=12, lateral=(0.2, 0.2, 0))
    run_fit(bottom, top)
    _result, rep, vol = merge_selected(bottom, top)
    assert rep["is_manifold"] and rep["shells"] == 1, rep
    assert abs(vol - 600) < 3, vol


def test_merge_closes_open_gap_and_seam():
    """Fit, then pull the top 0.15 away along the axis: a gap all the way
    round, open to the outside. With Fill Gaps at 2% (0.2 on these 10-unit
    blocks) Merge must close it, seam included."""
    reset()
    bottom, top = make_pair()
    displace(top, 20, 1, pull_back=12)
    run_fit(bottom, top)
    s = bpy.context.scene.assemble_by_points
    axis = (bottom.matrix_world.to_3x3() @ Vector(s.fit_axis)).normalized()
    top.matrix_world = Matrix.Translation(axis * 0.15) @ top.matrix_world
    s.gap_pct = 2.0
    result, rep, vol = merge_selected(bottom, top)
    depth = seam_depth(result, lambda x, y: interface(x, y) + 0.075)
    print(f"  seam: rays get {depth:.3f} past the outer wall")
    assert rep["is_manifold"] and rep["shells"] == 1, rep
    assert 600 < vol < 600 + 0.15 * 100 + 3, vol       # the gap got filled in
    assert depth < 0.04, depth                          # ~ the walls' own offset


def test_merge_dense_scans():
    """Scan-sized meshes (well over 20,000 edges each), like real scans."""
    reset()
    bottom = scan_piece("Bottom", -3.0, 160, 0.01, seed=1)
    top = scan_piece("Top", 3.0, 143, 0.01, seed=2)
    click_pairs(top, bottom, SOLID_SPOTS)
    assert len(bottom.data.edges) > 20000 and len(top.data.edges) > 20000
    displace(top, 20, 1, pull_back=12)
    run_fit(bottom, top)
    _result, rep, vol = merge_selected(bottom, top)
    assert rep["is_manifold"] and rep["shells"] == 1, rep
    assert abs(vol - 600) < 3, vol


def box(name, size, center):
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=Vector(size), verts=bm.verts)
    bmesh.ops.translate(bm, vec=Vector(center), verts=bm.verts)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def remember_fit(target, moving, axis=(0, 0, 1)):
    """Tell Merge which axis to close gaps along, as Fit Together would."""
    s = bpy.context.scene.assemble_by_points
    s.fit_target, s.fit_moving, s.fit_axis = target, moving, axis


def wall_x(result, z, y=0.3):
    """x of the merged object's +x wall at height z (ray fired from outside)."""
    from mathutils.bvhtree import BVHTree
    bvh = BVHTree.FromObject(result, bpy.context.evaluated_depsgraph_get())
    loc = bvh.ray_cast(Vector((10, y, z)), Vector((-1, 0, 0)))[0]
    return loc.x if loc is not None else None


def test_merge_never_welds_within_a_part():
    """A tiny detail far from the seam (a dent whose vertex is 0.01 from its
    neighbour) must survive a Merge Distance of 0.02: only vertices from
    different parts are merged."""
    reset()
    bottom, top = make_pair()
    bm = bmesh.new()
    bm.from_mesh(bottom.data)
    bm.transform(bottom.matrix_world)
    cap_edge = next(e for e in bm.edges
                    if all(v.co.z < -2.99 for v in e.verts) and abs(e.verts[0].co.x) < 1)
    _e, dent = bmesh.utils.edge_split(cap_edge, cap_edge.verts[0], 0.09)
    dent.co.z += 0.005          # a real (if tiny) detail: a dent 0.01 from a corner
    bm.transform(bottom.matrix_world.inverted())
    bm.to_mesh(bottom.data)
    bm.free()
    displace(top, 20, 1, pull_back=12)
    run_fit(bottom, top)
    bpy.context.scene.assemble_by_points.merge_pct = 0.2          # 0.02 on these blocks
    # compare positions: exactly coincident duplicates (split vertices, zero
    # distance apart) are the one thing that is collapsed within a part
    cap_before = {tuple(round(c, 4) for c in bottom.matrix_world @ v.co)
                  for v in bottom.data.vertices if (bottom.matrix_world @ v.co).z < -2.99}
    result, rep, _vol = merge_selected(bottom, top)
    cap_after = {tuple(round(c, 4) for c in v.co) for v in result.data.vertices if v.co.z < -2.99}
    print(f"  bottom cap: {len(cap_before)} vertex positions before, "
          f"{len(cap_before - cap_after)} missing after")
    print("  missing:", sorted(cap_before - cap_after))
    assert cap_before <= cap_after
    assert rep["is_manifold"] and rep["shells"] == 1, rep


def test_merge_leaves_detail_away_from_the_seam():
    """Every vertex farther than the smoothing width from the seam must still
    lie exactly on the original surface of one of the parts."""
    from mathutils.bvhtree import BVHTree
    reset()
    bottom, top = make_pair()
    displace(top, 20, 1, pull_back=12)
    run_fit(bottom, top)
    dg = bpy.context.evaluated_depsgraph_get()
    originals = []
    for obj in (bottom, top):
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        bm.transform(obj.matrix_world)
        originals.append(BVHTree.FromBMesh(bm))
        bm.free()
    width = bpy.context.scene.assemble_by_points.seam_pct / 100 * 10      # 10-unit blocks
    result, rep, _vol = merge_selected(bottom, top)
    far = [v.co for v in result.data.vertices
           if abs(v.co.z - interface(v.co.x, v.co.y)) > width + 0.3]
    off = max(min(bvh.find_nearest(co)[3] for bvh in originals) for co in far)
    print(f"  {len(far)} vertices away from the seam, furthest off the original: {off:.2e}")
    # 0.05% of the part, well below the scans' own noise (0.02): gap closing
    # moves along the assembly axis, which is a few degrees off true when it
    # comes from the plane through three clicked points on a wavy surface
    assert off < 5e-3, off


def test_merge_low_poly_boxes():
    """Two 8-vertex boxes, 0.1 apart and stepped 0.05 sideways: the faces
    have no vertices to bend with until Merge subdivides them. The gap must
    close and the step must become a smooth ramp; walls further away stay put."""
    reset()
    bottom = box("Bottom", (4, 4, 2), (0, 0, -1))
    top = box("Top", (4, 4, 2), (0.05, 0, 1.1))
    remember_fit(bottom, top)
    s = bpy.context.scene.assemble_by_points
    s.gap_pct, s.seam_pct = 3.0, 5.0            # boxes are ~4 units: 0.12 and 0.2
    result, rep, vol = merge_selected(bottom, top)
    assert rep["is_manifold"] and rep["shells"] == 1, rep
    assert abs(vol - (32 + 32 + 16 * 0.1)) < 0.5, vol      # the gap is filled in
    far_low, far_high = wall_x(result, -1.0), wall_x(result, 1.5)
    ramp = [wall_x(result, z) for z in (-0.1, 0.0, 0.05, 0.1, 0.2)]
    print(f"  wall x: far below {far_low:.4f}, far above {far_high:.4f}, across the seam "
          + ", ".join(f"{x:.4f}" for x in ramp))
    assert abs(far_low - 2.0) < 1e-4 and abs(far_high - 2.05) < 1e-4
    assert all(1.99 < x < 2.06 for x in ramp)
    assert 2.001 < ramp[2] < 2.049                          # mid-seam: part way, no step
    assert all(b >= a - 0.002 for a, b in zip(ramp, ramp[1:]))  # rises smoothly

    reset()                                                 # without smoothing: a step
    bottom = box("Bottom", (4, 4, 2), (0, 0, -1))
    top = box("Top", (4, 4, 2), (0.05, 0, 1.1))
    remember_fit(bottom, top)
    s.gap_pct, s.seam_pct = 3.0, 0.0
    result, rep, _vol = merge_selected(bottom, top)
    step = [wall_x(result, z) for z in (-0.1, 0.0, 0.05, 0.1, 0.2)]
    print("  without smoothing: " + ", ".join(f"{x:.4f}" for x in step))
    assert rep["is_manifold"] and rep["shells"] == 1, rep


def test_merge_seam_is_not_denser_than_the_merge_distance():
    """Defaults (1% = 0.1 on these blocks, scans ~0.11 apart): the merged mesh
    must not gain lots of vertices, and seam edges shorter than the merge
    distance must be rare."""
    reset()
    bottom, top = make_pair()
    displace(top, 20, 1, pull_back=12)
    run_fit(bottom, top)
    faces_in = len(bottom.data.polygons) + len(top.data.polygons)
    result, rep, _vol = merge_selected(bottom, top)
    me = result.data
    seam_edges = [e for e in me.edges
                  if all(abs(me.vertices[i].co.z - interface(me.vertices[i].co.x,
                                                             me.vertices[i].co.y)) < 0.1
                         for i in e.vertices)]
    short = sum(1 for e in seam_edges
                if (me.vertices[e.vertices[0]].co - me.vertices[e.vertices[1]].co).length < 0.05)
    print(f"  faces: {faces_in} in, {len(me.polygons)} out; seam edges {len(seam_edges)}, "
          f"{short} shorter than half the merge distance")
    assert rep["is_manifold"] and rep["shells"] == 1, rep
    assert len(me.polygons) < faces_in                 # the mating surfaces went inside
    assert short < 0.05 * max(len(seam_edges), 1), short


def test_merge_leaves_wide_gaps_alone():
    reset()
    bottom, top = make_pair()
    displace(top, 20, 1, pull_back=12)
    run_fit(bottom, top)
    s = bpy.context.scene.assemble_by_points
    axis = (bottom.matrix_world.to_3x3() @ Vector(s.fit_axis)).normalized()
    top.matrix_world = Matrix.Translation(axis * 1.0) @ top.matrix_world
    s.gap_pct = 3.0
    _result, rep, _vol = merge_selected(bottom, top)
    assert rep["shells"] == 2, rep                      # 1.0 gap: stays two pieces


def test_merge_hollow_parts_fills_the_cavity():
    """Joining two hollow halves encloses their cavity: an internal void,
    which Merge removes, so the result is solid."""
    reset()
    bottom, top = make_hollow_pair()
    click_pairs(top, bottom, RIM_SPOTS[:3])
    displace(top, 40, 1, pull_back=12)
    run_fit(bottom, top)
    _result, rep, vol = merge_selected(bottom, top)
    assert rep["is_manifold"] and rep["shells"] == 1, rep
    assert abs(vol - 600) < 4, vol                      # solid, cavity filled


def test_merge_scan_with_holes():
    reset()
    bottom, top = make_pair()
    # punch a hole in the bottom scan's outer wall, as scans often have
    bm = bmesh.new()
    bm.from_mesh(bottom.data)
    wall = [f for f in bm.faces if f.normal.x > 0.9][:3]
    bmesh.ops.delete(bm, geom=wall, context="FACES_ONLY")
    bm.to_mesh(bottom.data)
    bm.free()
    assert not merge.manifold_report(bottom.data)["is_manifold"]
    displace(top, 20, 1, pull_back=12)
    run_fit(bottom, top)
    _result, rep, vol = merge_selected(bottom, top)
    assert rep["is_manifold"] and rep["shells"] == 1, rep
    assert abs(vol - 600) < 3, vol


def test_pair_points_stay_on_the_surface():
    """Pair points follow the object and survive origin changes and applied
    transforms, which rewrite the mesh's local coordinates."""
    reset()
    bottom, top = make_pair(with_pairs=False)
    click_pairs(top, bottom, [(1.0, 1.0)], error=0)
    pair = bpy.context.scene.assemble_by_points.pairs[0]
    before = pair.world_points()[0].copy()
    local_before = top.matrix_world.inverted() @ before
    displace(top, 40, 3, offset=(1, 2, 10))
    moved = top.matrix_world @ local_before
    assert (pair.world_points()[0] - moved).length < 1e-4
    set_origin(top, Vector((3, 3, 3)))                  # like Blender's Set Origin
    top.data.transform(top.matrix_world)                # like Apply All Transforms
    top.matrix_world = Matrix.Identity(4)
    assert (pair.world_points()[0] - moved).length < 1e-4


def test_hollow_parts_with_matching_pairs():
    reset()
    bottom, top = make_hollow_pair()
    click_pairs(top, bottom, RIM_SPOTS[:3])
    truth = top.matrix_world.copy()
    displace(top, -150, 2, pull_back=15, lateral=(0.2, -0.3, 0))
    secs = run_fit(bottom, top, spin_range=math.radians(360))
    err = pose_error(top.matrix_world, truth)
    print(f"  pose error {err:.3f} in {secs:.1f}s")
    assert err < 0.1, err


def test_symmetric_hollow_box_is_oriented_by_the_pairs():
    """A square box with a flat rim looks the same every 90 degrees: the
    matching points decide which way round it goes."""
    flat = lambda x, y: 0.0
    reset()
    bottom, top = make_hollow_pair(surface=flat)
    click_pairs(top, bottom, RIM_SPOTS[:3], surface=flat)
    truth = top.matrix_world.copy()
    displace(top, 100, 1, pull_back=15, lateral=(0.1, 0.2, 0))
    run_fit(bottom, top, spin_range=math.radians(360))
    err = pose_error(top.matrix_world, truth)
    print(f"  pose error {err:.3f}")
    assert err < 0.15, err


def test_pair_tools():
    reset()
    bottom, top = make_pair(with_pairs=False)
    click_pairs(top, bottom, RIM_SPOTS)
    pairs = bpy.context.scene.assemble_by_points.pairs
    colours = {tuple(round(c, 3) for c in p.color) for p in pairs}
    assert len(pairs) == 4 and len(colours) == 4
    assert bpy.ops.assemble_by_points.remove_pair(index=1) == {"FINISHED"}
    assert len(pairs) == 3
    click_pairs(top, bottom, [(0, 0)] * 5)              # fill up to the limit of 8
    try:
        click_pairs(top, bottom, [(0, 0)])
        raise AssertionError("a 9th pair was accepted")
    except ValueError:
        pass
    assert bpy.ops.assemble_by_points.clear_pairs() == {"FINISHED"}
    assert len(pairs) == 0


def test_tools_are_off_outside_assembly_mode():
    reset()
    bottom, top = make_pair()
    select(bottom, top, active=bottom)
    bpy.context.scene.assemble_by_points.enabled = False
    assert not bpy.ops.assemble_by_points.fit.poll()
    assert not bpy.ops.assemble_by_points.merge.poll()
    assert not bpy.ops.assemble_by_points.check_manifold.poll()
    bpy.context.scene.assemble_by_points.enabled = True
    assert bpy.ops.assemble_by_points.fit.poll()
    assert bpy.ops.assemble_by_points.merge.poll()


def test_fit_needs_three_matching_points():
    reset()
    bottom, top = make_pair(with_pairs=False)
    select(bottom, top, active=bottom)
    click_pairs(top, bottom, SOLID_SPOTS[:2])
    assert not bpy.ops.assemble_by_points.fit.poll()              # button disabled with 2
    click_pairs(top, bottom, SOLID_SPOTS[2:])
    assert bpy.ops.assemble_by_points.fit.poll()                  # enabled with 3


def test_matching_points_in_a_line_are_an_error():
    reset()
    bottom, top = make_pair(with_pairs=False)
    click_pairs(top, bottom, [(-3.0, 0.0), (0.0, 0.0), (3.0, 0.0)], error=0)
    select(bottom, top, active=bottom)
    try:
        result = bpy.ops.assemble_by_points.fit()
    except RuntimeError as err:              # ERROR reports raise in background mode
        assert "in a line" in str(err), err
        result = {"CANCELLED"}
    assert result == {"CANCELLED"}


# ---------------------------------------------------------------- runner

def main():
    assemble_by_points.register()
    tests = [(k, v) for k, v in globals().items() if k.startswith("test_")]
    only = os.environ.get("ONLY")
    failed = 0
    for name, fn in tests:
        if only and only not in name:
            continue
        print(name)
        try:
            fn()
            print("  ok")
        except Exception:
            failed += 1
            traceback.print_exc()
            print("  FAILED")
    assemble_by_points.unregister()
    print(f"\n{'FAILED' if failed else 'all passed'} ({failed} failures)")
    sys.exit(1 if failed else 0)


main()
