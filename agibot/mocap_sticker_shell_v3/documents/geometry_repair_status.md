# Printable geometry verification

**Status: both fused STL halves pass geometry checks.** All ten obsolete
external bosses and bores are removed. Physical fit, printer-process validation,
hardware retention and crash testing have not been performed.

## Final exported meshes

| Check | Half A | Half B |
|---|---:|---:|
| Triangles | 83,340 | 102,652 |
| Connected components | 1 | 1 |
| Closed / consistently wound | Pass | Pass |
| Every edge used exactly twice | Pass | Pass |
| Duplicate / degenerate triangles | 0 / 0 | 0 / 0 |
| Independent transverse crossings | 0 | 0 |
| Independent coplanar overlaps | 0 | 0 |
| FreeCAD self-intersection pairs | 0 | 0 |
| FreeCAD solid / nonmanifold | Solid / false | Solid / false |

The finite-triangle checker recomputes normals from double-precision vertex
coordinates. It includes shared-vertex crossings and shared-edge coplanar
foldbacks; transverse tolerance is 1e-7 mm and coplanar area threshold 1e-8 mm².
These are finite-precision checks, not an exact-predicate mathematical proof.
The independent CAD application's checks also pass on the serialized STLs.
The two halves have zero assembled intersection volume in the mesh boolean test.

See [printability_validation.json](printability_validation.json) for SHA-256
identities, bounds, volumes and machine-readable results.

## How the obsolete features were removed

The old f1 boss was replaced using its original boundary and tangent-constrained
outer-skin continuation, with a bounded inner patch. The remaining front and
rear bosses use local exterior continuation patches following their surrounding
skin. The old external cones, top faces and screw bores are gone. Lower portions
of the old internal cavities were filled to restore a wall; upper reliefs and
ribs were retained instead of deleting deep internal structures.

The flat 12 mm dual-mode stations were then fused into each repaired half.
Their 3.4 mm central bores and captive-nut pockets remain. Structural shell
fasteners, seams and openings were excluded from the obsolete-feature deletion.

All **250 ray probes** through the ten old bore locations hit closed skin.
The former top planes are retracted by at least 2.38 mm in those probe regions.
All **24 new stations** independently pass checks on the final meshes for their
flat top planes, 12 mm outside diameter, 3.4 mm bore and 4.6 mm axial bore depth.
Their marker positions and axes are unchanged.

## Surface retention and thickness

Mesh cleanup used bounded simplification and a tiny local correction, not
global voxel reconstruction or broad automated hole filling. The rear cleanup
moved three final vertices by at most 0.006164 mm; that bound applies only to
that last adjustment, not the complete simplification.

Bidirectional samples of the cleanup stages found maximum incremental surface
distances of 0.029220 mm front and 0.037950 mm rear. These measurements cover
vertices, edge midpoints and triangle centres, not a certified Hausdorff bound.

A separate comparison used 27,474 front and 29,896 rear native CAD samples,
excluding the intentionally altered old-mount boxes and exterior station
footprints:

| Sampled comparison | Front | Rear |
|---|---:|---:|
| Archived mesh's absolute distance from original CAD, maximum | 0.108503 mm | 0.167367 mm |
| Final mesh's absolute distance from original CAD, maximum | 0.108503 mm | 0.167367 mm |
| Largest increase in distance at a compared point | 0.014470 mm | 0.016483 mm |
| Points whose distance increased by more than 0.05 mm | 0 | 0 |

The archived tessellation already exceeded its nominal 0.05 mm meshing setting
at some sampled locations. Therefore that setting is not presented as a
certified absolute tolerance. Near small fillets and sharp creases, nearest-facet
normals can change while positional changes remain tiny; not every original
CAD face or normal is claimed identical.

Restored-wall sampling found:

- f2-f5: minimum sampled clearance approximately 2.494-2.497 mm.
- b1-b5: minimum sampled clearance approximately 2.410-2.497 mm.
- f1: minimum local vertical gap approximately **1.503 mm**, with median
  approximately 2.530 mm. This includes its thinner boundary region.
- Probes specifically through the former bores: shortest sampled material
  path 2.293 mm.

These use different sampling methods and do not certify a global minimum
thickness or crash strength. Inspect sliced layers around the approximately
1.5 mm f1 edge and verify the printed part before robot use. Nominal CAD
registration is not a physical fit measurement.

## Why the earlier candidates were rejected

Earlier whole-body healing and broad mesh repair did not simultaneously
preserve surfaces and produce usable geometry. Native suppression of four
front internal cones also produced two malformed planar faces: shared vertices
were 0.635 mm and 0.514 mm off their declared planes, leaving large unmeshed
areas. Those candidates were rejected; the final meshes do not use them.

The original rear topology had genuine but shallow triangle crossings.
Its earlier raw CAD intersection count also included numerical or boundary
false positives. The final repair addressed confirmed geometry and was then
checked by both independent methods. No unresolved intersection count was
simply waived.

All failed conversions, trials, inspection scripts and intermediate meshes
remain in the local ignored engineering archive for recovery. Original source
files are unchanged. The authoritative original STEP SHA-256 is
`4ce4a322ddef39faaf80591dfb8a23e7365f0ff9813bac3330f6753bc5471992`.
