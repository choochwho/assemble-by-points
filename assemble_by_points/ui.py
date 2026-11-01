import math
import textwrap

import blf
import bpy
import gpu
from bpy.app.handlers import persistent
from bpy_extras import view3d_utils
from gpu_extras.batch import batch_for_shader

from . import operators
from .operators import assembly_mode
from .properties import MAX_PAIRS


INSTRUCTIONS = ("Face the parts the way they should join. Then assign 3 or more "
                "matching points to join them. The active part stays put; the other "
                "part moves.")


def _wrapped(layout, context, text, icon="NONE"):
    """Labels wrapped to the sidebar's current width."""
    scale = context.preferences.system.ui_scale
    width = context.region.width if context.region else 300
    chars = max(20, int((width - 40 * scale) / (6.5 * scale)))
    col = layout.column(align=True)
    for i, line in enumerate(textwrap.wrap(text, chars)):
        col.label(text=line, icon=icon if i == 0 else "BLANK1" if icon != "NONE" else "NONE")


class VIEW3D_PT_assemble_by_points(bpy.types.Panel):
    bl_label = "Assembly Mode"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Assemble by Points"

    def draw_header(self, context):
        self.layout.prop(context.scene.assemble_by_points, "enabled", text="")

    def draw(self, context):
        s = context.scene.assemble_by_points
        layout = self.layout
        if not s.enabled:
            layout.label(text="Tick the box to use the assembly tools.")
            return

        _wrapped(layout, context, INSTRUCTIONS, icon="INFO")

        settings = layout.column()
        settings.use_property_split = True
        settings.use_property_decorate = False
        settings.prop(s, "tilt_limit")
        settings.prop(s, "spin_range")

        box = layout.box()
        box.label(text="Matching Points", icon="STICKY_UVS_LOC")
        pairs = s.pairs
        box.operator("assemble_by_points.add_pair", icon="ADD",
                     text=f"Create Matching Points ({len(pairs)}/{MAX_PAIRS})")
        col = box.column(align=True)
        for i, pair in enumerate(pairs):
            row = col.row(align=True)
            sw = row.row()
            sw.ui_units_x = 1.2
            sw.prop(pair, "color", text="")
            a = pair.obj_a.name if pair.obj_a else "(deleted)"
            b = pair.obj_b.name if pair.obj_b else "(deleted)"
            label = f"{i + 1}  {a} ↔ {b}"
            if pair.obj_a and pair.obj_b and pair.world_points() is None:
                label += "  (mesh changed)"
            row.label(text=label)
            row.operator("assemble_by_points.remove_pair", text="", icon="X").index = i
        if pairs:
            box.operator("assemble_by_points.clear_pairs", icon="TRASH")
        if draw_error:
            err = box.box()
            err.alert = True
            err.label(text="Couldn't draw the points:", icon="ERROR")
            err.label(text=draw_error)

        layout.operator("assemble_by_points.fit", icon="SNAP_ON")


class VIEW3D_PT_assemble_by_points_merge(bpy.types.Panel):
    bl_label = "Merge"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Assemble by Points"

    @classmethod
    def poll(cls, context):
        return assembly_mode(context)

    def draw(self, context):
        s = context.scene.assemble_by_points
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False
        layout.prop(s, "merge_pct")
        layout.prop(s, "gap_pct")
        layout.prop(s, "seam_pct")
        meshes = [o for o in context.selected_objects if o.type == "MESH"]
        if len(meshes) >= 2:
            unit = context.scene.unit_settings
            fmt = lambda v: bpy.utils.units.to_string(unit.system, "LENGTH",
                                                      v * unit.scale_length, precision=3)
            size = operators.assembled_size(meshes)
            spacing, gap, seam = operators.merge_settings(context, meshes)
            _wrapped(layout, context,
                     f"The selected parts span {fmt(size)} (the assembled size once "
                     f"they're fitted), so 1% = {fmt(size / 100)}: merge {fmt(spacing)}, "
                     f"gaps {fmt(gap)}, seam {fmt(seam)}.")
        layout.operator("assemble_by_points.merge", icon="MOD_BOOLEAN")
        layout.operator("assemble_by_points.check_manifold", icon="CHECKMARK")


# --------------------------------------------------------------------------
# Viewport overlay: the matching points. The draw
# hooks only exist while Assembly Mode is on in the current scene.
# --------------------------------------------------------------------------

_handles = []

# Last error raised while drawing the overlay (shown in the panel, because
# Blender only prints draw-callback errors to the hidden system console).
draw_error = None

DOT_RADIUS = 7.0        # pixels
OUTLINE = 2.5           # dark ring around each dot, pixels
LINE_WIDTH = 3.0


def _pair_points(context):
    """[(point on a, point on b, colour, number, show a, show b)] per usable pair.

    Each pair is handled on its own, so one broken pair (deleted object,
    edited mesh, object not in this view layer) can't hide the others.
    """
    global draw_error
    out = []
    for i, pair in enumerate(context.scene.assemble_by_points.pairs):
        try:
            pts = pair.world_points()
            if pts is None:
                continue
            view_layer = context.view_layer
            show_a = pair.obj_a.visible_get(view_layer=view_layer)
            show_b = pair.obj_b.visible_get(view_layer=view_layer)
            out.append((pts[0], pts[1], tuple(pair.color), i + 1, show_a, show_b))
        except Exception as err:                    # noqa: BLE001 - never break drawing
            draw_error = f"pair {i + 1}: {err}"
    return out


def _dots(context):
    """[(world point, colour, number)] for every visible matching point."""
    dots = []
    for pa, pb, color, number, show_a, show_b in _pair_points(context):
        if show_a:
            dots.append((pa, color, number))
        if show_b:
            dots.append((pb, color, number))
    if operators.pending_point is not None:
        dots.append(operators.pending_point)
    return dots


def _draw_3d():
    """Lines (in 3D, always on top) linking the two points of each pair."""
    global draw_error
    context = bpy.context
    if not assembly_mode(context):
        return
    try:
        lines, colors = [], []
        for pa, pb, color, _n, show_a, show_b in _pair_points(context):
            if show_a and show_b:
                lines += [pa[:], pb[:]]
                colors += [(*color, 0.9)] * 2
        if not lines:
            return
        depth, blend = gpu.state.depth_test_get(), gpu.state.blend_get()
        gpu.state.depth_test_set("NONE")
        gpu.state.blend_set("ALPHA")
        shader = gpu.shader.from_builtin("POLYLINE_FLAT_COLOR")
        shader.uniform_float("viewportSize", gpu.state.viewport_get()[2:])
        shader.uniform_float("lineWidth", LINE_WIDTH)
        batch_for_shader(shader, "LINES", {"pos": lines, "color": colors}).draw(shader)
        gpu.state.depth_test_set(depth)
        gpu.state.blend_set(blend)
    except Exception as err:                        # noqa: BLE001
        draw_error = f"lines: {err}"


def _disc(cx, cy, radius, segments=20):
    pts = [(cx, cy)]
    for k in range(segments + 1):
        ang = 2 * math.pi * k / segments
        pts.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang)))
    return pts


def _draw_2d():
    """Dots and numbers, drawn in screen space so they're round and a fixed size."""
    global draw_error
    context = bpy.context
    if not assembly_mode(context):
        return
    try:
        region, rv3d = context.region, context.region_data
        spots = []
        for co, color, number in _dots(context):
            xy = view3d_utils.location_3d_to_region_2d(region, rv3d, co)
            if xy is not None:
                spots.append((xy, color, number))
        if not spots:
            return
        blend = gpu.state.blend_get()
        gpu.state.blend_set("ALPHA")
        shader = gpu.shader.from_builtin("UNIFORM_COLOR")
        for xy, color, _n in spots:
            for radius, rgba in ((DOT_RADIUS + OUTLINE, (0.0, 0.0, 0.0, 0.9)),
                                 (DOT_RADIUS, (*color, 1.0))):
                shader.uniform_float("color", rgba)
                batch_for_shader(shader, "TRI_FAN", {"pos": _disc(xy.x, xy.y, radius)}).draw(shader)
        gpu.state.blend_set(blend)

        font = 0
        blf.size(font, 16)
        blf.enable(font, blf.SHADOW)
        blf.shadow(font, 3, 0.0, 0.0, 0.0, 1.0)
        for xy, color, number in spots:
            blf.position(font, xy.x + DOT_RADIUS + 4, xy.y + DOT_RADIUS - 2, 0)
            blf.color(font, *color, 1.0)
            blf.draw(font, str(number))
        blf.disable(font, blf.SHADOW)
    except Exception as err:                        # noqa: BLE001
        draw_error = f"dots: {err}"


def _redraw_viewports():
    wm = getattr(bpy.context, "window_manager", None)
    for window in getattr(wm, "windows", []):
        for area in window.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()


def sync_axis_drawing():
    """Add or remove the draw hooks to match Assembly Mode."""
    want = not bpy.app.background and assembly_mode(bpy.context)
    if want and not _handles:
        _handles.append(bpy.types.SpaceView3D.draw_handler_add(
            _draw_3d, (), "WINDOW", "POST_VIEW"))
        _handles.append(bpy.types.SpaceView3D.draw_handler_add(
            _draw_2d, (), "WINDOW", "POST_PIXEL"))
    elif not want:
        _remove_handles()
    _redraw_viewports()


def _remove_handles():
    while _handles:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(_handles.pop(), "WINDOW")
        except (ValueError, RuntimeError):
            pass                    # Blender already freed it (see _resync_drawing)


def _resync_drawing():
    """Drop and re-add the draw hooks. Blender's Reload Scripts discards every
    Python draw hook *after* add-ons have registered again, so hooks added
    during register silently stop drawing; this runs just after, from a timer.
    """
    try:
        _remove_handles()
        sync_axis_drawing()
    except AttributeError:          # still no usable context; nothing to draw into
        pass
    return None                     # run once


@persistent
def _on_load(_dummy):
    sync_axis_drawing()


classes = (VIEW3D_PT_assemble_by_points, VIEW3D_PT_assemble_by_points_merge)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.app.handlers.load_post.append(_on_load)
    try:
        sync_axis_drawing()
    except AttributeError:          # restricted context while Blender starts up
        pass
    if not bpy.app.background:
        bpy.app.timers.register(_resync_drawing, first_interval=0.25)


def unregister():
    if bpy.app.timers.is_registered(_resync_drawing):
        bpy.app.timers.unregister(_resync_drawing)
    if _on_load in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load)
    _remove_handles()
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
