"""The normal HOPE contact selection from MujocoSimModule.

The MJCF compiles both contact choices into its BVH; the running simulator
disables the vendor sole spheres and keeps the ankle hulls. Loading the XML
without this step enables both and does not represent either runtime profile.
"""

import mujoco


def apply_hope_contact_profile(model):
    for side in ("left", "right"):
        for point in range(1, 14):
            index = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_foot{point}_collision"
            )
            if index >= 0:
                model.geom_contype[index] = 0
                model.geom_conaffinity[index] = 0
