"""Monte-Carlo ball-landing analysis (2026-07-07, run as: python -m hope_planner.landing_mc): does a policy that matches the commanded racket
state (within measured error) actually put the ball ON THE TABLE?

Pipeline per sample (the exact deploy chain, using hope_planner's venue-fitted physics):
  1. sample a plausible lob ARRIVAL state at the strike plane (pos in the trained per-clip
     bands, descending incoming velocity, mild spin)
  2. planner.plan() -> commanded racket state (v_racket, n_racket) via the drag shooting
     solve + impact model (identical code to deploy)
  3. perturb the command by the POLICY'S MEASURED execution error (vel-norm sigma, normal
     angle sigma, strike-point offset)
  4. forward contact model predict_paddle_contact() (same venue fit as training virtual_ball)
  5. integrate outgoing flight (quadratic drag): net clearance at net_x, landing point at
     z=0 (table surface) -> classify good / net / long / wide / short
"""
import numpy as np
from hope_planner import BallPhysics, PlannerConfig, TableParams, RacketTargetPlanner
from hope_planner import predict_paddle_contact
from hope_planner.ball_trajectory_predictor import StrikeTarget

rng = np.random.default_rng(7)
physics, config, table = BallPhysics(), PlannerConfig(), TableParams()
planner = RacketTargetPlanner(physics, config, table)

ROBOT_Y = -0.7625          # robot mid-table on the forehand half (arena §9.3)
FH_Y = (-0.48, -0.40)      # v13 facefix arm-reach y bands, station-relative (receipt contacts
BH_Y = (-0.13, -0.05)      # fh -0.441 / bh -0.082; disjoint around the planner split -0.25)
FH_Z = (0.09, 0.54)        # v13 trained z [0.85,1.30] (floor) minus table height 0.76
BH_Z = (0.09, 0.54)

def sample_strike(clip):
    x = rng.uniform(0.02, 0.35)                    # arena x_hit clamp band
    yb = FH_Y if clip == 0 else BH_Y
    zb = FH_Z if clip == 0 else BH_Z
    y = np.clip(ROBOT_Y + rng.uniform(*yb), -table.width + 0.03, -0.03)
    z = rng.uniform(*zb)
    p = np.array([x, y, z])
    v = np.array([rng.uniform(-4.0, -1.5), rng.uniform(-0.6, 0.6), rng.uniform(-2.5, -0.2)])
    ax = rng.normal(size=3); ax[0] = 0.0; ax /= (np.linalg.norm(ax) + 1e-9)
    omega = rng.uniform(0.0, 30.0) * ax            # mild residual spin after the bounce
    return p, v, omega

def rot_about(axis, ang):
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)

def perturb(v_r, n, sig_v, sig_n_deg, rng):
    v = v_r + rng.normal(0.0, sig_v / np.sqrt(3.0), 3)
    perp = np.cross(n, rng.normal(size=3)); perp /= (np.linalg.norm(perp) + 1e-12)
    ang = abs(rng.normal(0.0, np.deg2rad(sig_n_deg)))
    return v, rot_about(perp, ang) @ n

def land_point(p0, v0):
    """Integrate drag flight; return (landing_xy, z_at_net, hit_net_plane)."""
    p, v = p0.astype(float).copy(), v0.astype(float).copy()
    dt = 0.002
    z_net = None
    for _ in range(3000):
        a = -physics.k * np.linalg.norm(v) * v + physics.g
        p2, v2 = p + v * dt + 0.5 * a * dt * dt, v + a * dt
        if z_net is None and (p[0] - table.net_x) * (p2[0] - table.net_x) <= 0 and v[0] > 0:
            al = 0 if abs(p2[0] - p[0]) < 1e-12 else (table.net_x - p[0]) / (p2[0] - p[0])
            z_net = (p + al * (p2 - p))[2]
        if p2[2] <= 0.0 and v2[2] < 0:
            al = 0 if abs(p2[2] - p[2]) < 1e-12 else (0.0 - p[2]) / (p2[2] - p[2])
            return p + al * (p2 - p), z_net
        p, v = p2, v2
    return None, z_net

def run(sig_v, sig_n_deg, sig_p, n_samples=4000, label=""):
    stats = dict(good=0, net=0, long=0, wide=0, short=0, no_plan=0)
    lands = []
    for i in range(n_samples):
        clip = i % 2
        p_s, v_in, om = sample_strike(clip)
        cmd = planner.plan(StrikeTarget(p_ball=p_s, v_ball=v_in, t_strike=1.5,
                                        num_bounces=1, valid=True))
        if not (cmd.valid and cmd.clears_net):
            stats["no_plan"] += 1; continue
        v_r, n_r = perturb(cmd.v_racket, cmd.n_racket, sig_v, sig_n_deg, rng)
        p_c = p_s + rng.normal(0.0, sig_p / np.sqrt(3.0), 3)     # strike-point error
        v_plus, _ = predict_paddle_contact(v_in, v_r, n_r, om, physics, config)
        land, z_net = land_point(p_c, v_plus)
        if v_plus[0] <= 0.2 or land is None:
            stats["net"] += 1; continue
        if z_net is not None and z_net <= table.net_height + 0.005:
            stats["net"] += 1; continue
        x, y = land[0], land[1]
        if x <= table.net_x:  stats["short"] += 1
        elif x > table.length: stats["long"] += 1
        elif not (-table.width <= y <= 0.0): stats["wide"] += 1
        else: stats["good"] += 1; lands.append((x, y))
    att = n_samples - stats["no_plan"]
    lands = np.array(lands) if lands else np.zeros((0, 2))
    on = stats["good"] / max(att, 1)
    print(f"{label:>28} | plan-ok {att*100//n_samples:>3}% | ON-TABLE {on*100:5.1f}% | "
          f"net {stats['net']/max(att,1)*100:4.1f}% long {stats['long']/max(att,1)*100:4.1f}% "
          f"wide {stats['wide']/max(att,1)*100:4.1f}% short {stats['short']/max(att,1)*100:4.1f}%"
          + (f" | land mean ({lands[:,0].mean():.2f},{lands[:,1].mean():+.2f}) "
             f"std ({lands[:,0].std():.2f},{lands[:,1].std():.2f})" if len(lands) else ""))
    return on

def main():
    print(f"target_land = {config.target_land}, table x[0,{table.length}] y[{-table.width},0], "
          f"net @x={table.net_x} h={table.net_height}\n")
    print(f"{'error level':>28} |")
    run(0.0, 0.0, 0.0,   label="PERFECT execution (sanity)")
    run(0.17, 4.0, 0.025, label="champion 12900 (meas.)")
    run(0.25, 6.0, 0.035, label="typical eval spread")
    run(0.35, 10.0, 0.05, label="degraded")
    run(0.5, 15.0, 0.075, label="TRAINING PASS THRESHOLDS")


if __name__ == "__main__":
    main()
