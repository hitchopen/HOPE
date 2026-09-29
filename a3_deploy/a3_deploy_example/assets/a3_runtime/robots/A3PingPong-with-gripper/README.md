# A3 ping-pong model with OP3 gripper

Operator-supplied URDF and its referenced meshes. All mesh URLs are relative to the URDF, so this package can be moved with its directory structure intact.

Mesh filename case is corrected for Linux. The supplied package did not contain 20 gripper collision meshes; those collision entries now explicitly reference the corresponding supplied visual meshes. This is a loading fallback, not validated collision geometry. The duplicated pelvis IMU link declaration is deduplicated, retaining the declaration that references its supplied collision mesh. Joint transforms, unique-link inertias and limits are preserved. The linkage has no calibrated closed-loop jaw motion; do not infer a held-ball center from a linkage origin.

The held-ball release site and physical release delay still need calibration. This URDF alone does not establish reliable ball interception or collision safety.
