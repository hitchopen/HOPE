"""Ball-physics calibration (HOPE planner step 8).

Fit the drag coefficient k and the horizontal/vertical restitution
coefficients C_h, C_v from recorded ball trajectories, following
HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md, Section 2.1.

`calibrate_ball_physics()` is the pure-numpy fitter from the reference doc.
A small CLI (`hope_calibrate`) loads recorded trajectories from CSV files
(one file per trajectory: columns t,x,y,z) and prints the fitted parameters.
"""

import argparse
import csv
from typing import List

import numpy as np

from .constants import BallPhysics


def calibrate_ball_physics(
    trajectories: List[np.ndarray],
    timestamps: List[np.ndarray],
    g: np.ndarray = np.array([0.0, 0.0, -9.81]),
) -> BallPhysics:
    """Calibrate k, C_h, C_v from recorded ball trajectories.

    Parameters
    ----------
    trajectories : list of (N, 3) arrays
        Each array is a sequence of ball positions [x, y, z] in the HOPE frame.
    timestamps : list of (N,) arrays
        Corresponding timestamps in seconds.

    Returns
    -------
    BallPhysics with fitted parameters.
    """
    drag_a = []
    drag_v2 = []
    h_restitution = []
    v_restitution = []

    for pos, t in zip(trajectories, timestamps):
        if len(t) < 4:
            continue
        dt = np.diff(t)
        vel = np.diff(pos, axis=0) / dt[:, None]
        t_vel = 0.5 * (t[:-1] + t[1:])
        dt_vel = np.diff(t_vel)
        acc = np.diff(vel, axis=0) / dt_vel[:, None]
        vel_mid = 0.5 * (vel[:-1] + vel[1:])
        z_mid = 0.5 * (pos[:-2, 2] + pos[2:, 2])

        # Flight samples: z well above the table.
        flight_mask = z_mid > 0.03
        if np.any(flight_mask):
            a_flight = acc[flight_mask]
            v_flight = vel_mid[flight_mask]
            a_minus_g = a_flight - g[None, :]
            drag_a.extend(np.linalg.norm(a_minus_g, axis=1))
            drag_v2.extend(np.linalg.norm(v_flight, axis=1) ** 2)

        # Bounce detection: three-sample pattern (descend -> contact -> rise).
        z = pos[:, 2]
        for i in range(1, len(z) - 1):
            if z[i - 1] > 0.01 and z[i] < 0.005 and z[i + 1] > 0.01:
                idx_pre = max(0, i - 2)
                idx_post = min(len(vel) - 1, i + 1)
                if idx_pre < len(vel) and idx_post < len(vel):
                    v_pre = vel[idx_pre]
                    v_post = vel[idx_post]
                    v_h_pre = np.hypot(v_pre[0], v_pre[1])
                    v_h_post = np.hypot(v_post[0], v_post[1])
                    if v_h_pre > 0.1:
                        h_restitution.append(v_h_post / v_h_pre)
                    if abs(v_pre[2]) > 0.1:
                        v_restitution.append(abs(v_post[2]) / abs(v_pre[2]))

    # Fit drag: |a - g| = k * |v|^2  (least squares through the origin).
    if drag_a:
        a_mag = np.array(drag_a)
        v2 = np.array(drag_v2)
        k = float(np.dot(a_mag, v2) / np.dot(v2, v2))
    else:
        k = 0.1261   # venue-fit default (see constants.BallPhysics)

    C_h = float(np.median(h_restitution)) if h_restitution else 0.64
    C_v = float(np.median(v_restitution)) if v_restitution else 0.9215

    return BallPhysics(k=k, C_h=C_h, C_v=C_v)


def load_trajectory_csv(path: str):
    """Load a single trajectory CSV with columns t,x,y,z (header optional)."""
    t_list, p_list = [], []
    with open(path, newline="") as fh:
        reader = csv.reader(fh)
        for row in reader:
            if not row or row[0].strip().lower() in ("t", "time"):
                continue  # skip header / blank
            t_list.append(float(row[0]))
            p_list.append([float(row[1]), float(row[2]), float(row[3])])
    return np.array(t_list), np.array(p_list)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Calibrate HOPE ball physics (k, C_h, C_v) from recorded CSV trajectories."
    )
    parser.add_argument(
        "csv", nargs="+",
        help="One or more trajectory CSV files (columns: t,x,y,z in the HOPE frame).",
    )
    args = parser.parse_args(argv)

    trajectories, timestamps = [], []
    for path in args.csv:
        t, p = load_trajectory_csv(path)
        timestamps.append(t)
        trajectories.append(p)

    phys = calibrate_ball_physics(trajectories, timestamps)
    print("Calibrated ball physics:")
    print(f"  drag_k        = {phys.k:.4f}")
    print(f"  restitution_h = {phys.C_h:.4f}")
    print(f"  restitution_v = {phys.C_v:.4f}")
    print("\nPaste into config/hope_planner.yaml:")
    print(f"    drag_k: {phys.k:.4f}")
    print(f"    restitution_h: {phys.C_h:.4f}")
    print(f"    restitution_v: {phys.C_v:.4f}")


if __name__ == "__main__":
    main()
