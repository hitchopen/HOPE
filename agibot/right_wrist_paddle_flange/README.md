# A3 right wrist paddle flange

The current printable 45-degree right wrist paddle mount has a hollow connector
cavity and smooth reinforcement across its external wall. The inner mating
profile, mounting planes, four base screw passages, four hand-side ring screw
passages, screw-head seats and two locating pegs are retained.

## Release files

| File | Use |
| --- | --- |
| [45deg_right_wrist_flange_mm.stl](45deg_right_wrist_flange_mm.stl) | Printable manufacturing master with smooth exterior reinforcement. |
| [45deg_right_wrist_flange_print_notes.txt](45deg_right_wrist_flange_print_notes.txt) | Printing, geometry verification, fit checks and source hash records. |

## Printing and fit

Import the STL in **millimetres at 100% scale**. Its original X/Y/Z dimensions
are approximately **45.724 × 63.052 × 46.548 mm**. The lowest original Z is
−1.500 mm; use the slicer's place-on-bed operation before slicing.

The exterior expands outward by up to 2.5 mm, with gradual transitions near the
base and screw recesses. The nominal circular wall at the hand-side opening is
approximately 5 mm; thickness varies around the keyed profile and screw access.
Keep the modeled hollow cavity and screw passages open when preparing the print.
See the print notes for supports, scale checks and assembly details.

The released mesh passed the recorded watertight/manifold and self-intersection
checks. Sampled inner-socket differences from the hollow reference were below
0.001 mm, and screw/tool clearance checks passed. Physical fit and impact
strength still require testing, including clearance around the enlarged body
throughout the wrist's motion range.

STL SHA-256:
`1449b23c0820cdb5b5eaac32c47b997a193e9557aeb2bfceb7e4464f4bfd5592`.

See the [Agibot folder index](../README.md) for the robot assets and serving
fixture, and [mocap_sticker_shell_v3](../mocap_sticker_shell_v3/README.md) for
the pelvis marker shell, Motive asset and marker-to-`pelvis_link` tables.
