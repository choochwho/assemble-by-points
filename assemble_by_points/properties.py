import math

import bpy
from bpy.props import (BoolProperty, CollectionProperty, FloatProperty,
                       FloatVectorProperty, IntVectorProperty, PointerProperty)

MAX_PAIRS = 8
PAIR_COLORS = (
    (1.00, 0.25, 0.25),   # red
    (1.00, 0.70, 0.10),   # amber
    (0.35, 0.90, 0.25),   # green
    (0.20, 0.85, 1.00),   # cyan
    (0.35, 0.45, 1.00),   # blue
    (0.75, 0.35, 1.00),   # purple
    (1.00, 0.35, 0.80),   # pink
    (1.00, 1.00, 1.00),   # white
)


def _mode_changed(self, context):
    from . import ui
    ui.sync_axis_drawing()


def _is_mesh(self, obj):
    return obj.type == "MESH"


def _anchor_point(obj, verts, weights):
    """World position of a surface anchor, or None if the mesh changed under it."""
    mv = obj.data.vertices
    if max(verts) >= len(mv):
        return None
    local = sum((mv[i].co * w for i, w in zip(verts, weights)), mv[verts[0]].co * 0)
    return obj.matrix_world @ local


class MatchPair(bpy.types.PropertyGroup):
    """Two points, one on each object, that touch when the parts are assembled.

    Each point is anchored to the mesh surface (three vertices plus barycentric
    weights), so it follows the object when it moves and survives origin
    changes and applied transforms; only editing the mesh topology breaks it.
    """
    obj_a: PointerProperty(type=bpy.types.Object, poll=_is_mesh)
    verts_a: IntVectorProperty(size=3)
    weights_a: FloatVectorProperty(size=3)
    obj_b: PointerProperty(type=bpy.types.Object, poll=_is_mesh)
    verts_b: IntVectorProperty(size=3)
    weights_b: FloatVectorProperty(size=3)
    color: FloatVectorProperty(name="Colour", subtype="COLOR_GAMMA", size=3,
                               min=0.0, max=1.0)

    def world_points(self):
        """(point on obj_a, point on obj_b), or None if either can't be found."""
        if self.obj_a is None or self.obj_b is None:
            return None
        pa = _anchor_point(self.obj_a, self.verts_a, self.weights_a)
        pb = _anchor_point(self.obj_b, self.verts_b, self.weights_b)
        return None if pa is None or pb is None else (pa, pb)


class AssemblySettings(bpy.types.PropertyGroup):
    enabled: BoolProperty(
        name="Assembly Mode",
        description="Turn the assembly tools on. While off, the add-on draws nothing "
                    "and its tools are disabled",
        default=False, update=_mode_changed,
    )
    tilt_limit: FloatProperty(
        name="Tilt Limit",
        description="How far the moving object may tip away from the axis of assembly "
                    "(the line between the two origins)",
        default=math.radians(5), min=0.0, max=math.radians(30),
        subtype="ANGLE", precision=1,
    )
    spin_range: FloatProperty(
        name="Spin Range",
        description="How far the moving object may turn about the axis of assembly, "
                    "centred on how it faces now (180 = up to 90 either way, "
                    "360 = any orientation)",
        default=math.radians(180), min=0.0, max=math.radians(360),
        subtype="ANGLE", precision=0,
    )
    pairs: CollectionProperty(type=MatchPair)

    # Merge settings are percentages of the assembled parts' size (the largest
    # dimension of their combined bounding box), so they suit any scale.
    merge_pct: FloatProperty(
        name="Merge Distance",
        description="Percent of the assembled size. In the seam area, vertices end up "
                    "about this far apart and never closer; no part is subdivided finer "
                    "than this. Vertices away from the seam are never merged",
        default=1.0, min=0.0, soft_max=5.0, max=100.0, subtype="PERCENTAGE", precision=2,
    )
    gap_pct: FloatProperty(
        name="Fill Gaps Up To",
        description="Percent of the assembled size. Gaps between the fitted parts up to "
                    "this wide are closed when merging, measured along the axis of "
                    "assembly (slightly wider gaps are partly closed). 0 = don't close gaps",
        default=1.0, min=0.0, soft_max=5.0, max=100.0, subtype="PERCENTAGE", precision=2,
    )
    seam_pct: FloatProperty(
        name="Smooth Seam",
        description="Percent of the assembled size. The two parts' walls are blended "
                    "together across a band this wide on each side of the seam. Nothing "
                    "outside the band is changed. 0 = don't smooth",
        default=1.0, min=0.0, soft_max=5.0, max=100.0, subtype="PERCENTAGE", precision=2,
    )

    # Remembered from the last Fit, so Merge can close gaps along the same
    # axis even once the parts (and maybe their origins) sit together.
    fit_target: PointerProperty(type=bpy.types.Object)
    fit_moving: PointerProperty(type=bpy.types.Object)
    fit_axis: FloatVectorProperty(size=3, description="In the target's local space")


classes = (MatchPair, AssemblySettings)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.assemble_by_points = bpy.props.PointerProperty(type=AssemblySettings)


def unregister():
    del bpy.types.Scene.assemble_by_points
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
