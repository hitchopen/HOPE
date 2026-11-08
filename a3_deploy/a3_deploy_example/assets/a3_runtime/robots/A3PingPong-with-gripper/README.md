# A3 ping-pong model with OP3 gripper

Use [the 0807 model](urdf/model.urdf) with the adjacent `meshes/` directory. It has 31 movable body joints. The left OP3 linkage is fixed at its zero pose. Each visual and collision pair references the same local STL.

Both elbow joint origins are 10 mm forward along the parent frame's X axis. Both ankle pitch joint origins have zero lateral offset. Link names are case-sensitive, including `right_wrist_yaw_Link`.

Jaw motion, the held-ball release site, physical release delay and collision geometry still require hardware validation.
