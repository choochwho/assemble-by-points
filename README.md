# Assemble by Points (Blender extension)

Fits two scanned meshes together (SLS, photogrammetry) and merges them into one manifold mesh, ready for 3D printing.
You mark a few matching points on the two parts, and the add-on finds the exact fit.
The meshes never match exactly, so the fit doesn't rely on vertices or faces lining up.
Instead it looks for the pose with the lowest average gap/overlap between the two surfaces.

It works on Blender 4.2 and later and has been tested on 5.2.

## Install

1. Download `assemble_by_points-<version>.zip` from the repository's **Releases** page.
2. In Blender, go to **Edit → Preferences → Get Extensions → ▾ (top right) → Install from Disk…** and pick the zip.

The tools appear in the 3D Viewport sidebar (**N**) under the **Assemble by Points** tab.

## Workflow

Tick **Assembly Mode** at the top of the Assemble by Points tab. While it's off, the add-on draws nothing and its tools are disabled, so it stays out of the way of normal work.

1. **Face the parts the way they should join.** The distance between them doesn't matter.
2. **Assign 3 or more matching points.** A matching point is a pair of spots, one on each part, that touch when the parts are assembled. See [Matching points](#matching-points).
3. **Select both parts.** The *active* one (selected last) stays put, and the other one moves.
4. Click **Fit Together**. It stays disabled until both parts are selected and at least 3 matching points link them.
5. Optionally, click **Merge Into New Object**.

## Options

| Option | Default | Meaning |
|---|---|---|
| **Tilt Limit** | 5° | How far the moving part may tip away from the axis of assembly. The maximum is 30°. |
| **Spin Range** | 180° | How far the moving part may turn about the axis of assembly, centred on its current orientation. 180° means up to 90° either way, and 360° allows any orientation. |

The **axis of assembly**, the direction the parts join along, comes from the matching points:
- If turning the moving part's points onto the target's points is a real turn (over about 20°), that turn's axis is used. That's exactly the joining direction, so no extra tilt is needed however far the part has to spin.
- Otherwise, the axis is the direction perpendicular to the plane through the target's points.

The centre of each part's matching points is where the parts meet. The moving part can also slide sideways by up to 30% of its radius.

After a fit, the status bar reports:
- the average gap/overlap where the parts touch
- how much of the footprint touches
- how far apart the matching points ended up

Each fit is a single undo step.

- The target's matching points must not lie in a line. Spread them around the joint.
- The finest search grid is 1/192 of the moving part's footprint diameter, which is about 0.08 on a 15-unit part.

## Matching points

- **Create Matching Points** (in the Assembly Mode panel): click a spot on one object, and a numbered, coloured dot appears. Then click the spot on the other object that it touches.
  Each press of the button makes one pair, up to 8. Clicking empty space, Esc, or right-click cancels.
- The panel lists every pair with its colour. The **✕** button removes one pair, and **Clear All Pairs** removes them all.
- Each dot is anchored to the mesh surface (to a face's vertices), not to the object's origin. It stays in place when you move objects, change their origins, or apply transforms.
  It only breaks if you edit the mesh's topology, and then the panel shows "(mesh changed)".
- **Fit Together** needs at least 3 pairs linking the two selected objects. They set the axis of assembly and where the parts meet, and every candidate pose is also scored on how far apart its pairs end up.
  Clicks within about 2% of the part's radius count as exact, so small click errors don't pull the fit off.
  The pairs mostly decide the coarse spin and slide, and the surfaces decide the final seating.
  Afterwards, the status bar reports how far apart the pairs ended up.
- Spread the pairs around the part, for example 3 points far apart on a rim.

## Merge Into New Object

Select the fitted parts and click **Merge Into New Object**. The result is a **new** object made of one closed outer shell with no internal geometry, ready for resin printing.
The originals are hidden, never modified.

All three settings are **percentages of the assembled size**, which is the largest dimension of the selected parts' combined bounding box. They suit any scale: for parts 25 m across, 1% is 0.25 m.
When two or more meshes are selected, the panel shows the assembled size and what each setting comes to.

| Option | Default | Meaning |
|---|---|---|
| **Merge Distance** | 1% | The vertex spacing target for the seam area. Merge never subdivides finer than this, and it collapses crowded seam vertices (for example the Boolean's slivers) without thinning the seam below the density of the mesh around it. Away from the seam, nothing is ever merged. |
| **Fill Gaps Up To** | 1% | Gaps between the fitted parts up to this wide are closed. Gaps up to 1.5× this are partly closed, and wider ones are left alone. 0 turns gap closing off. |
| **Smooth Seam** | 1% | The width of the band on each side of the seam that gets blended, so the two parts flow into each other. 0 turns smoothing off. |

What it does, in order, on copies of the parts:
1. **Prepares each part.** It collapses only exactly coincident duplicate vertices (split vertices in scan data) and fills any holes in the scans.
2. **Closes the gaps.** It uses the same axis of assembly the last **Fit Together** used.
   - First it subdivides mating faces that are coarser than the gap size, so even low-poly parts have vertices to move. Dense scans are left as they are.
   - Every vertex on each mating surface then measures the gap to the other part along the axis, and both sides move halfway across it, plus a little so they overlap.
   - At the outer seam, the rim edges bend to meet.
3. **Unions** the parts with a Boolean.
4. **Cleans up.** It deletes internal voids (cavities enclosed inside the solid) and slivers, fills any holes, and re-fills tangled spots.
5. **Evens out the seam band.**
   - Edges there that are longer than twice the merge distance are split.
   - Edges shorter than half of either the merge distance or the mesh's own spacing, whichever is smaller, are collapsed. That removes slivers without making the seam coarser than its surroundings.
   - Nothing outside the band is touched.
6. **Smooths the seam.**
   - Each vertex in it is moved onto a blend of the two parts' outer walls: all of one part's wall at one edge of the band, all of the other part's at the other edge, and smoothly in between.
   - A small step or notch between the parts becomes a gentle ramp.
   - The areas where the parts touched are inside the solid by now, so they're never affected.
   - Where the two walls are further apart than the band width, for example a lid that really overhangs its base, the blend is skipped.

Anything that moves a vertex first splits the long edges running away from it. A move only tilts small faces next to it, so detail outside the seam band stays put: in the tests, within 0.003 on a 10-unit part (0.03%).

**Hollow parts become solid.** When two hollow halves join, the space inside them is an enclosed void, so Merge fills it. That's what resin printing needs, but it uses more resin than a drained, hollow print would.

**Check Manifold** reports problems on any selected meshes.

## How the fit works

The code is in `assemble_by_points/fitting.py`.

1. **Height fields.** Rays are cast along the axis on a grid. They record the target's surface that faces the moving part, and the moving part's surface that faces the target.
2. **Scoring a pose.** A pose is a spin about the axis plus a sideways slide. For each pose, the gap or overlap is known at every grid cell.
   Sliding along the axis shifts every gap equally, so the best axial position comes out in closed form. So does a small tilt, which adds a plane to the heights.
   The fit minimises the mean |gap/overlap| plus the matching-pair distances.
   - Gaps wider than 4 grid cells are capped as "not touching".
   - Overlap keeps costing up to 10× longer, because solids can't pass through each other. Without this, the inside of a hollow part could "touch" the inside of the other while their rims pass through each other.
   - Parts of the footprint that don't overlap count against the pose.
3. **Coarse to fine.**
   - Every spin and slide is scored on a 24-cell grid.
   - The best 6 candidates are refined on 48-, 96- and 192-cell grids.
   - The winner is polished by re-casting the moving part's rays at its exact tilt and pattern-searching spin and slide.
   - Tilt limits above 5° add explicitly cast tilted orientations in rings of 10°.

## Development

```
assemble_by_points/        the extension package (this is what gets built and installed)
  blender_manifest.toml    id, version, name, Blender version
  __init__.py              register/unregister, plus reloading for F3 > Reload Scripts
  fitting.py               the fit: height fields, coarse-to-fine search
  merge.py                 the merge: gap closing, boolean union, cleanup, seam smoothing
  operators.py             the tools (bpy.ops.assemble_by_points.*)
  properties.py            scene settings and matching pairs (scene.assemble_by_points)
  ui.py                    the sidebar panels and the viewport overlay
tests/run_tests.py         headless tests
```

To work on it with live reloading, link the package folder into Blender's extensions folder instead of installing the zip. On Windows, run this in a Command Prompt from the repository folder:

```
mklink /J "%APPDATA%\Blender Foundation\Blender\5.2\extensions\user_default\assemble_by_points" assemble_by_points
```

Then enable **Assemble by Points** under **Preferences → Add-ons**, and press **F3 → Reload Scripts** after each change.
Don't uninstall the extension from Preferences while the link exists. Delete the link itself instead.

## Tests

```
blender --background --factory-startup --python tests/run_tests.py
```

Set `ONLY=<part of a test name>` to run a subset.

The tests use synthetic "scans": two blocks that meet along a wavy surface with a 0.5-unit peg and a groove.
Each side has its own tessellation and 0.02 surface noise. Three matching points are clicked slightly off, and the moving part starts far away, spun and tilted.

The tests cover:
- recovery with default settings, a full 360° spin, and a 12° tilt with the limit raised
- hollow cups (0.5-unit walls, so only the rims meet) with matching pairs
- a symmetric square box that only matching pairs can orient correctly
- pair points staying on the surface through moves, origin changes, and applied transforms
- the pair tools (colours, remove, clear, the limit of 8)
- the spin range being respected
- a part laid out off to the side with a 150° spin, at the default tilt limit
- merging into a single manifold shell
- closing a gap that is open all the way round, including the outer seam, while leaving wide gaps alone
- filling the enclosed cavity of hollow parts
- repairing a scan with holes
- never merging vertices within a part (a tiny dent far from the seam survives)
- leaving detail away from the seam in place
- low-poly parts: the gap closes and a step between the walls becomes a smooth ramp
- the seam not getting denser than the merge distance, and the merged mesh not gaining faces
- Fit Together staying disabled with fewer than 3 matching points
- the error when the matching points lie in a line

## Build

```
blender --command extension build --source-dir assemble_by_points --output-dir dist
```

Bump `version` in `blender_manifest.toml` first. The zip in `dist/` is what gets attached to a GitHub release.

## License

GPL-3.0-or-later, the same as Blender. See [LICENSE](LICENSE).
