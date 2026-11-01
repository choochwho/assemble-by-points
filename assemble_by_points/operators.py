import math
import time

import bpy
from bpy_extras import view3d_utils
from mathutils import Matrix, Vector, geometry

from . import fitting, merge
from .properties import MAX_PAIRS, PAIR_COLORS


def _selected_meshes(context):
    return [o for o in context.selected_objects if o.type == "MESH"]


def assembly_mode(context):
    scene = getattr(context, "scene", None)
    return scene is not None and scene.assemble_by_points.enabled


def _mode_poll(cls, context):
    """Shared poll: every tool is off unless Assembly Mode is on."""
    if not assembly_mode(context):
        cls.poll_message_set("Turn on Assembly Mode in the Assemble by Points tab")
        return False
    return True


def fit_pair(context):
    """(target, moving) when exactly two meshes are selected and one is active."""
    active = context.active_object
    meshes = _selected_meshes(context)
    if active and active.type == "MESH" and len(meshes) == 2 and active in meshes:
        return active, next(o for o in meshes if o is not active)
    return None


def assembled_size(objects):
    """Largest dimension of the objects' combined world bounding box: the
    reference that the merge percentages are taken of."""
    corners = [o.matrix_world @ Vector(c) for o in objects for c in o.bound_box]
    if not corners:
        return 0.0
    return max(max(c[i] for c in corners) - min(c[i] for c in corners) for i in range(3))


def merge_settings(context, objects):
    """(merge distance, max gap, seam width) in scene units for these objects."""
    s = context.scene.assemble_by_points
    size = assembled_size(objects)
    return tuple(size * pct / 100 for pct in (s.merge_pct, s.gap_pct, s.seam_pct))


class _ObjectModeScope:
    """Leave edit mode for the duration, then restore it."""

    def __init__(self, context):
        self.was_edit = context.mode == "EDIT_MESH"

    def __enter__(self):
        if self.was_edit:
            bpy.ops.object.mode_set(mode="OBJECT")
        return self

    def __exit__(self, *exc):
        if self.was_edit:
            bpy.ops.object.mode_set(mode="EDIT")
        return False


class ASSEMBLE_BY_POINTS_OT_fit(bpy.types.Operator):
    """Move the selected part onto the active one: the matching points set where it
    goes, and the surfaces decide the exact spin, tilt and position"""
    bl_idname = "assemble_by_points.fit"
    bl_label = "Fit Together"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        if not _mode_poll(cls, context):
            return False
        pair = fit_pair(context)
        if pair is None:
            cls.poll_message_set("Select two mesh objects; the active one stays fixed")
            return False
        count = len(pairs_for(context.scene, pair[1], pair[0]))
        if count < fitting.MIN_PAIRS:
            cls.poll_message_set(f"Create at least {fitting.MIN_PAIRS} matching points "
                                 f"between the two parts ({count} so far)")
            return False
        return True

    def execute(self, context):
        s = context.scene.assemble_by_points
        target, moving = fit_pair(context)
        pairs = pairs_for(context.scene, moving, target)
        wm = context.window_manager
        t0 = time.perf_counter()
        with _ObjectModeScope(context):
            wm.progress_begin(0, 100)
            try:
                result = fitting.fit(target, moving, context.evaluated_depsgraph_get(),
                                     s.tilt_limit, s.spin_range, pairs=pairs,
                                     progress=lambda f: wm.progress_update(int(f * 100)))
            except ValueError as err:
                self.report({"ERROR"}, str(err))
                return {"CANCELLED"}
            finally:
                wm.progress_end()
            moving.matrix_world = Matrix(result.matrix.tolist()) @ moving.matrix_world
            s.fit_target, s.fit_moving = target, moving
            s.fit_axis = target.matrix_world.to_3x3().inverted() @ Vector(result.axis)

        unit = context.scene.unit_settings
        fmt = lambda v: bpy.utils.units.to_string(unit.system, "LENGTH", v * unit.scale_length,
                                                  precision=3)
        self.report({"INFO"},
                    f"Fit in {time.perf_counter() - t0:.1f}s: average gap/overlap "
                    f"{fmt(result.mean_gap)} over {result.contact:.0%} contact "
                    f"({result.overlap_share:.0%} of it overlapping). "
                    f"Spun {math.degrees(result.spin):.1f}°, tilted "
                    f"{math.degrees(result.tilt):.1f}°, slid {fmt(result.shift)} sideways"
                    + (f". {len(pairs)} matching pair(s): on average "
                       f"{fmt(sum(result.pair_distances) / len(pairs))} apart, worst "
                       f"{fmt(max(result.pair_distances))}" if pairs else ""))
        return {"FINISHED"}


class _ClickTool:
    """Modal plumbing shared by the click-on-a-mesh tools.

    Left-clicks in the viewport are ray cast into the scene and handed to
    ``on_hit``. Clicking empty space or outside the viewport, Esc, right-click
    or leaving Assembly Mode all cancel, so the tool never keeps swallowing
    clicks. Everything else passes through, so navigation keeps working.
    """
    bl_options = {"REGISTER", "UNDO"}

    def status(self):
        raise NotImplementedError

    def on_hit(self, context, obj, location, face_index):
        """Return "FINISHED", "CANCELLED" or "RUNNING_MODAL"."""
        raise NotImplementedError

    def on_end(self, context):
        pass

    def invoke(self, context, event):
        if context.mode != "OBJECT":
            self.report({"WARNING"}, "Switch to Object Mode first")
            return {"CANCELLED"}
        context.window_manager.modal_handler_add(self)
        context.window.cursor_modal_set("CROSSHAIR")
        context.workspace.status_text_set(self.status())
        return {"RUNNING_MODAL"}

    def _end(self, context, result):
        self.on_end(context)
        context.window.cursor_modal_restore()
        context.workspace.status_text_set(None)
        for area in context.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()
        return result

    def modal(self, context, event):
        if not assembly_mode(context) or context.area is None:
            return self._end(context, {"CANCELLED"})
        if event.type in {"ESC", "RIGHTMOUSE"} and event.value == "PRESS":
            return self._end(context, {"CANCELLED"})
        if event.type != "LEFTMOUSE" or event.value != "PRESS":
            return {"PASS_THROUGH"}

        region = next((r for r in context.area.regions if r.type == "WINDOW"
                       and r.x <= event.mouse_x < r.x + r.width
                       and r.y <= event.mouse_y < r.y + r.height), None)
        if region is None:                  # clicked outside the viewport itself
            return self._end(context, {"CANCELLED", "PASS_THROUGH"})
        rv3d = context.area.spaces.active.region_3d
        coord = (event.mouse_x - region.x, event.mouse_y - region.y)
        origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
        direction = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
        hit, location, _n, index, obj, _m = context.scene.ray_cast(
            context.evaluated_depsgraph_get(), origin, direction)
        if not hit or obj.type != "MESH":
            self.report({"WARNING"}, "No mesh under the cursor; cancelled")
            return self._end(context, {"CANCELLED"})

        result = self.on_hit(context, obj.original, location, index)
        if result == "RUNNING_MODAL":
            context.workspace.status_text_set(self.status())
            context.area.tag_redraw()
            return {"RUNNING_MODAL"}
        return self._end(context, {result})


# The first point of a pair while the second is being picked, for drawing:
# (world location, colour, pair number) or None.
pending_point = None


def _next_color(scene):
    used = {tuple(round(c, 3) for c in p.color) for p in scene.assemble_by_points.pairs}
    return next((c for c in PAIR_COLORS if tuple(round(x, 3) for x in c) not in used),
                PAIR_COLORS[len(scene.assemble_by_points.pairs) % len(PAIR_COLORS)])


def surface_anchor(obj, world_location, face_index=None):
    """Anchor a point on ``obj``'s surface: (vertex indices, barycentric weights).

    ``face_index`` is the hit polygon from a ray cast. Ray casts see the
    evaluated mesh, so if modifiers changed the topology the index may not
    match the base mesh; then the nearest surface point is used instead.
    """
    mesh = obj.data
    local = obj.matrix_world.inverted() @ world_location
    poly = None
    if face_index is not None and 0 <= face_index < len(mesh.polygons):
        poly = mesh.polygons[face_index]
        size = max((mesh.vertices[i].co - poly.center).length for i in poly.vertices)
        if abs((local - poly.center).dot(poly.normal)) > 0.05 * size:
            poly = None
    if poly is None:
        ok, snapped, _n, index = obj.closest_point_on_mesh(local)
        if not ok or index >= len(mesh.polygons):
            raise ValueError(f"Couldn't find the surface of '{obj.name}' there")
        local, poly = snapped, mesh.polygons[index]

    # Fan-triangulate the polygon and take the triangle containing the point.
    idx = list(poly.vertices)
    best = None
    for k in range(1, len(idx) - 1):
        tri = (idx[0], idx[k], idx[k + 1])
        a, b, c = (mesh.vertices[i].co for i in tri)
        w = geometry.barycentric_transform(local, a, b, c, Vector((1, 0, 0)),
                                           Vector((0, 1, 0)), Vector((0, 0, 1)))
        if best is None or min(w) > best[0]:
            best = (min(w), tri, w)
    _worst, tri, w = best
    w = [max(x, 0.0) for x in w]
    total = sum(w) or 1.0
    return tri, [x / total for x in w]


def add_pair(scene, obj_a, world_a, obj_b, world_b, face_a=None, face_b=None):
    """Store a matching pair (points given in world space) and return it."""
    s = scene.assemble_by_points
    if len(s.pairs) >= MAX_PAIRS:
        raise ValueError(f"At most {MAX_PAIRS} matching pairs")
    verts_a, weights_a = surface_anchor(obj_a, world_a, face_a)
    verts_b, weights_b = surface_anchor(obj_b, world_b, face_b)
    color = _next_color(scene)
    pair = s.pairs.add()
    pair.obj_a, pair.verts_a, pair.weights_a = obj_a, verts_a, weights_a
    pair.obj_b, pair.verts_b, pair.weights_b = obj_b, verts_b, weights_b
    pair.color = color
    return pair


def pairs_for(scene, moving, target):
    """World-space (moving point, target point) for pairs linking these objects."""
    out = []
    for pair in scene.assemble_by_points.pairs:
        pts = pair.world_points()
        if pts is None:
            continue
        if pair.obj_a == moving and pair.obj_b == target:
            out.append(pts)
        elif pair.obj_a == target and pair.obj_b == moving:
            out.append(pts[::-1])
    return out


class ASSEMBLE_BY_POINTS_OT_add_pair(_ClickTool, bpy.types.Operator):
    """Click a point on one object, then the point on the other object that it
    touches when the parts are assembled. Fit Together favours these pairs"""
    bl_idname = "assemble_by_points.add_pair"
    bl_label = "Create Matching Points"

    @classmethod
    def poll(cls, context):
        if not _mode_poll(cls, context):
            return False
        if context.area is None or context.area.type != "VIEW_3D":
            return False
        if len(context.scene.assemble_by_points.pairs) >= MAX_PAIRS:
            cls.poll_message_set(f"Already {MAX_PAIRS} pairs; remove one first")
            return False
        return True

    def invoke(self, context, event):
        self.first = None
        self.number = len(context.scene.assemble_by_points.pairs) + 1
        return super().invoke(context, event)

    def status(self):
        step = ("click a point on the first object" if self.first is None
                else "click the matching point on the OTHER object")
        return f"Matching pair {self.number}: {step}  •  Esc / right-click to cancel"

    def on_hit(self, context, obj, location, face_index):
        global pending_point
        if self.first is None:
            self.first = (obj, location.copy(), face_index)
            pending_point = (location.copy(), _next_color(context.scene), self.number)
            return "RUNNING_MODAL"
        if obj == self.first[0]:
            self.report({"WARNING"}, "Pick the matching point on the other object")
            return "RUNNING_MODAL"
        try:
            add_pair(context.scene, self.first[0], self.first[1], obj, location,
                     self.first[2], face_index)
        except ValueError as err:
            self.report({"ERROR"}, str(err))
            return "CANCELLED"
        self.report({"INFO"}, f"Added matching pair {self.number}: "
                              f"'{self.first[0].name}' ↔ '{obj.name}'")
        return "FINISHED"

    def on_end(self, context):
        global pending_point
        pending_point = None


class ASSEMBLE_BY_POINTS_OT_remove_pair(bpy.types.Operator):
    """Remove this matching pair"""
    bl_idname = "assemble_by_points.remove_pair"
    bl_label = "Remove Matching Pair"
    bl_options = {"REGISTER", "UNDO", "INTERNAL"}

    index: bpy.props.IntProperty()

    @classmethod
    def poll(cls, context):
        return _mode_poll(cls, context)

    def execute(self, context):
        pairs = context.scene.assemble_by_points.pairs
        if 0 <= self.index < len(pairs):
            pairs.remove(self.index)
        return {"FINISHED"}


class ASSEMBLE_BY_POINTS_OT_clear_pairs(bpy.types.Operator):
    """Remove all matching pairs"""
    bl_idname = "assemble_by_points.clear_pairs"
    bl_label = "Clear All Pairs"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return _mode_poll(cls, context) and len(context.scene.assemble_by_points.pairs) > 0

    def execute(self, context):
        context.scene.assemble_by_points.pairs.clear()
        return {"FINISHED"}


class ASSEMBLE_BY_POINTS_OT_merge(bpy.types.Operator):
    """Merge the selected objects into one new, closed mesh: close small gaps along
    the axis of assembly, union them, and remove internal voids"""
    bl_idname = "assemble_by_points.merge"
    bl_label = "Merge Into New Object"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        if not _mode_poll(cls, context):
            return False
        if len(_selected_meshes(context)) < 2:
            cls.poll_message_set("Select at least two mesh objects")
            return False
        return True

    def execute(self, context):
        s = context.scene.assemble_by_points
        t0 = time.perf_counter()
        with _ObjectModeScope(context):
            objects = _selected_meshes(context)
            active = context.active_object
            if active in objects:                    # active first: it names the result
                objects.remove(active)
                objects.insert(0, active)

            gap_pair, axis = self._gap_axis(context, objects)
            spacing, max_gap, seam_width = merge_settings(context, objects)
            try:
                result, report = merge.merge(objects, context, spacing, max_gap,
                                             seam_width=seam_width, gap_pair=gap_pair,
                                             axis=axis)
            except ValueError as err:
                self.report({"ERROR"}, str(err))
                return {"CANCELLED"}

            for o in objects:
                o.select_set(False)
                o.hide_set(True)
            result.select_set(True)
            context.view_layer.objects.active = result

        notes = []
        if report["holes_filled"]:
            notes.append(f"filled {report['holes_filled']} hole(s) in the input scans")
        if report["gap_vertices_moved"]:
            notes.append(f"closed gaps at {report['gap_vertices_moved']} vertices")
        elif gap_pair is None and s.gap_pct > 0:
            notes.append("gaps not closed (use Fit Together on these parts first)")
        if report["edges_split"] or report["seam_split"]:
            notes.append(f"split {report['edges_split'] + report['seam_split']} coarse edges")
        if report["seam_collapsed"]:
            notes.append(f"merged {report['seam_collapsed']} crowded vertices at the seam")
        if report["seam_smoothed"]:
            notes.append(f"smoothed the seam ({report['seam_smoothed']} vertices)")
        if report["voids_removed"]:
            notes.append(f"removed {report['voids_removed']} internal void(s)")
        if report["faces_repaired"]:
            notes.append(f"repaired {report['faces_repaired']} faces")
        ok = report["is_manifold"] and report["shells"] == 1
        state = "one closed, printable shell" if ok else merge.describe(report)
        if report["shells"] > 1:
            state += f"; {report['shells']} separate pieces (the parts don't touch everywhere)"
        msg = (f"Created '{result.name}' in {time.perf_counter() - t0:.1f}s "
               f"({len(result.data.polygons)} faces): {state}")
        if notes:
            msg += ". " + "; ".join(notes).capitalize()
        self.report({"INFO"} if ok else {"WARNING"}, msg)
        return {"FINISHED"}

    @staticmethod
    def _gap_axis(context, objects):
        """((target, moving), world axis) to close gaps along, or (None, None).

        That's the axis the last Fit Together used on these parts.
        """
        s = context.scene.assemble_by_points
        t, m = s.fit_target, s.fit_moving
        if t in objects and m in objects and Vector(s.fit_axis).length > 0:
            return (t, m), (t.matrix_world.to_3x3() @ Vector(s.fit_axis)).normalized()
        return None, None


class ASSEMBLE_BY_POINTS_OT_check_manifold(bpy.types.Operator):
    """Report whether each selected mesh is a closed manifold"""
    bl_idname = "assemble_by_points.check_manifold"
    bl_label = "Check Manifold"

    @classmethod
    def poll(cls, context):
        return _mode_poll(cls, context) and bool(_selected_meshes(context))

    def execute(self, context):
        with _ObjectModeScope(context):
            lines = [f"{o.name}: {merge.describe(merge.manifold_report(o.data))}"
                     for o in _selected_meshes(context)]
        all_ok = all(line.endswith(": manifold") for line in lines)
        self.report({"INFO"} if all_ok else {"WARNING"}, " | ".join(lines))
        return {"FINISHED"}


classes = (ASSEMBLE_BY_POINTS_OT_fit, ASSEMBLE_BY_POINTS_OT_add_pair,
           ASSEMBLE_BY_POINTS_OT_remove_pair, ASSEMBLE_BY_POINTS_OT_clear_pairs, ASSEMBLE_BY_POINTS_OT_merge,
           ASSEMBLE_BY_POINTS_OT_check_manifold)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
