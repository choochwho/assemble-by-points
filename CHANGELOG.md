# Changelog

## 0.9.0

First public release. Earlier versions were developed privately under the name "Assembly Fit".

- **Fit Together** moves one scanned part onto another. Three or more matching points set where the parts meet and which way they join, and the surfaces decide the exact spin, tilt and position.
- **Merge Into New Object** combines the fitted parts into one closed, printable shell. It closes small gaps, removes internal voids, and smooths the seam.
- **Matching points** are colour-coded and anchored to the mesh surface, so they survive moves, origin changes and applied transforms.
- The extension ID is `assemble_by_points`, and the tools are `bpy.ops.assemble_by_points.*`. Settings and matching points saved in .blend files by the old "Assembly Fit" add-on aren't carried over.
