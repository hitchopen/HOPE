# A3 v3 marker registration to pelvis_link

Status: **nominal CAD calculation verified; physical calibration not performed**.
Date: 2026-09-09. This note accompanies the 24-station sticker and mount-seat
tables. The fused STL geometry is now checked; this note does not certify
physical fit or installed calibration.

## Coordinate mapping

The source STEP uses millimetres. Its CAD Y axis becomes the robot Z axis.
For a point `[X, Y, Z]` in that unchanged CAD frame:

```text
pelvis_link_x_m = Z / 1000
pelvis_link_y_m = X / 1000
pelvis_link_z_m = (Y - 145.800095037096) / 1000

R_pelvis_link_from_CAD = [[0,0,1], [1,0,0], [0,1,0]]
t_pelvis_link_m       = [0,0,-0.145800095037096]
```

This is a proper rotation (determinant +1), not a reflection or a scale fit.
In homogeneous-transform notation, `T_pelvis_link_marker` transforms points
from the local marker frame into the parent `pelvis_link` frame. The inverse
transform is **not** what the supplied tables contain.

## How the registration was determined

The STEP itself does not label a robot root frame. The reference ten-marker
coordinate table supplies that frame, and an existing calibration record
explicitly identifies those same nominal coordinates as `pelvis_link`.
The old live P1-to-pelvis calibration transform was **not** reused.

Ten original socket-end annular faces were read directly from the latest STEP.
Each is a plane with area 42.223005264247 mm² and concentric circle radii 1.6 and
4.0 mm. The corresponding nominal optical centres have an unknown common
stand-off `h` along the socket's outward normal. We jointly solved:

```text
q_reference_mm = R * (p_socket_CAD_mm + h_mm * normal_CAD) + t_mm
```

All seven parameters—three rotation, three translation and the stand-off—were
fitted from 30 scalar coordinates. The initial rotation came from a proper
rigid alignment of socket centres; no final axis mapping or stand-off was
imposed. Different stand-off initial values (-10, 0, 6 and 20 mm) converged to
the same solution. Fitted `h = 6.000000000000 mm` applies only to the original
datums; **it is not a dimension for the new ball hardware**.

The following CAD positions are rounded here to six decimal places in mm.
Face numbers are one-based within each original source solid. The normal is
`[0,0,+1]` for front datums and `[0,0,-1]` for rear datums, to numerical precision.
Front/back and left/right assignments follow the reference coordinate identities;
the shell's symmetric original pattern is not being used as an unlabelled pose cue.

| Datum | Solid / face | Socket CAD X | Socket CAD Y | Socket CAD Z | Reference pelvis X | Reference pelvis Y | Reference pelvis Z |
|---|---|---:|---:|---:|---:|---:|---:|
| f1 | 1 / 1139 | 0 | 15.800095 | 84 | 90 | 0 | -130 |
| f2 | 1 / 1134 | 50 | 5.800095 | 74 | 80 | 50 | -140 |
| f3 | 1 / 1141 | -50 | 5.800095 | 74 | 80 | -50 | -140 |
| f4 | 1 / 1135 | -30 | -34.199905 | 72 | 78 | -30 | -180 |
| f5 | 1 / 1132 | 30 | -34.199905 | 72 | 78 | 30 | -180 |
| b1 | 2 / 1402 | 0 | 45.800095 | -84 | -90 | 0 | -100 |
| b2 | 2 / 1393 | 55 | 15.800095 | -79 | -85 | 55 | -130 |
| b3 | 2 / 1397 | -55 | 15.800095 | -79 | -85 | -55 | -130 |
| b4 | 2 / 1399 | -30 | -34.199905 | -79 | -85 | -30 | -180 |
| b5 | 2 / 1396 | 30 | -34.199905 | -79 | -85 | 30 | -180 |

## Verification results

### Flat-face revision, 2026-09-09

The revised additions STEP uses flat **12.0 mm** annular station tops with
**3.4 mm** central screw bores and small flush hole pins. It has no raised
guards or full-face sticker recesses. All 24 annular top centres, outward
normals, matching bores and flush pin tops were independently checked after
STEP reimport. Maximum centre discrepancy remains **9.995e-12 mm**; no station
material projects above the seat plane. Positions, normals and tangent axes
match the previous layout exactly. The CSV/XLSX numeric values are unchanged.

Current additions STEP SHA-256:
`d22491a9f849d26a2495798cdf26dd2abe22f210eba499e3b31f4df50a58a5d8`.
Current construction-layout SHA-256:
`24da11aa0a430cb8a124dd106f3ac03fac321293d806261c6f00cb0e330390f6`.

The original ten socket datums above are archived registration evidence,
**not marker mounts to retain on the redesigned shell**. Their removal does
not change this coordinate mapping if the shell frame and new stations stay
fixed. All ten obsolete external mounts and bores are now removed in the
geometry-checked STL masters. Final-mesh checks reconfirm all 24 planar tops
and open mounting bores, without moving their origins or axes. Physical fit
and manufacturing-process validation remain outstanding. The historical
verification and hashes below describe the earlier
large-plug revision, not the current flat-top hardware.

### Original registration and previous additions cross-check

| Check | Result |
|---|---|
| Maximum fitted datum residual | 4.122e-13 mm |
| RMS fitted datum residual | 2.026e-13 mm |
| Maximum leave-one-out prediction residual | 4.673e-13 mm; each of ten datums held out once |
| Fit Jacobian rank | 7 of 7 |
| Rotation determinant | 0.9999999999999993; published exact permutation has determinant 1 |
| Numerical acceptance threshold | 1e-6 mm |
| New marker-seat STEP check | 24 of 24 unique 12.6 mm circular plug-top faces located |
| Maximum new seat-centre discrepancy | 9.995e-12 mm |
| New outward normals | Dot product with STEP face normal within 1e-10 of 1 |
| Workbook vs independent calculation | All positions, normals and quaternions agree within 1e-12 |
| Film-thickness sensitivity | +0.10 mm moves sticker centres by 0.10 mm along each normal; mount seats unchanged; restored to 0.20 mm |

These very small residuals indicate shared nominal CAD geometry and numerical
agreement. **They do not imply sub-micrometre physical accuracy.** The reference
coordinates are nominal, the robot mesh is simplified, and no installed v3
shell or tracking measurements have been used to certify physical accuracy.

## New point and orientation definitions

For each station, the rigid mount-seat origin is the concentric screw axis at
the flat 12 mm top plane. Its nominal sticker optical origin is
`seat + 0.20 * n` in CAD mm.
The 0.20 mm film-plus-adhesive thickness was approved by the user, not measured.
The current additions STEP was independently reimported, and all 24 annular
top centres and outward normals were checked against these construction inputs.

Local axes are the CAD station's right-handed `[u, v, n]` basis. The published
quaternion comes from `R_pelvis_link_from_CAD * [u v n]`, using `qx,qy,qz,qw`
order and non-negative `qw`. A round passive marker does not measure its own
roll; this orientation is a useful mounting-frame convention.

CSV positions have nine decimal places in metres for deterministic interchange,
not as a manufacturing-accuracy claim. Quaternions and normals have twelve
decimal places. The workbook retains underlying precision and formula-driven
translation calculations. Changing registration or station geometry requires
rerunning the CAD checks and quaternion derivation; do not edit it as though
it were a validated live calibration tool.

For a ball with measured installed centre offset `h_ball_mm`:

```text
p_ball_pelvis_link_m = p_mount_seat_pelvis_link_m
                    + (h_ball_mm / 1000) * normal_pelvis_link
```

If the mounting stack is eccentric, measure a full local 3D offset instead.
The ball sphere diameter alone does not determine the installed offset. No
ball optical-centre table is supplied until that dimension is known. Switching
between sticker and ball mode requires the appropriate optical-centre table
and a new or independently validated Motive rigid-body calibration.

## Source identity and scope

Original source records and applicable license/attribution notices remain
unchanged. Engineering intermediates and reproducibility tools are retained
locally in the ignored archive, not packaged as printable release files.
Exact SHA-256 identities used for this calculation:

| Source | SHA-256 |
|---|---|
| Original shell STEP, latest Onshape export | `4ce4a322ddef39faaf80591dfb8a23e7365f0ff9813bac3330f6753bc5471992` |
| New dual-mode additions STEP | `10af73fe04074566a8b41d8b71b0f937a3c5088fb3962676dd863e2a7eeb1f7d` |
| New station-layout construction data | `c31c98a8fb6998acfc5d5c82d0f94837f28eb2f68ef469a9aaea28a6c9dfb236` |
| Original nominal ten-marker coordinate table document | `26849ff652aece88f4c9f38661f8e98a08b80df8f74d6bb7f425c00bbb0861b5` |
| Existing frame-identity receipt, 2026-08-05 | `47a66d19e99d1940ec4edd9d06ae8b0c3dab08262397e54bf6add2ea541d55a0` |

The original STEP and earlier full-assembly exports are not manufacturing
masters. The repaired, fused STL halves now pass independent topology and
intersection checks; their hashes and scope are in printability_validation.json.
This registration is valid for the checked nominal station placement in the
unchanged source coordinate system. Any
repair that changes datums, fit surfaces or station positions requires a new
comparison and, where relevant, updated tables. Physical fit, marker retention,
joint clearance, impact survival and live tracking remain unverified.
