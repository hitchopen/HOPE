"""PHYSICAL ball + table for the tracking task — Phase A/B TRUTH INSTRUMENT (metrics only).

WHAT THIS IS: a flag-gated (``RacketTargetCommandCfg.physical_ball`` /
task racket config / task-YAML ``racket.physical_ball``) real PhysX rigid-body ball, one per env,
plus a visual-only table USD, that realizes each swing's question-bank incoming ball physically.
Neither asset has a PhysX contact path to the robot:

* SERVE — when the per-swing question is known (bank resample) and ``time_to_strike`` enters the
  serve horizon, the ball is launched from the venue-model REVERSE-TIME integrated state
  (:func:`back_integrate_incoming`: RK4 of the fitted flight law with a negative step, TRUNCATED
  at the table plane — the FINAL BALLISTIC SEGMENT only), ``tts_effective`` seconds before the
  strike, so that forward flight arrives at the question's contact point with the question's
  incoming velocity exactly at the exact-strike frame.
* FLIGHT — PhysX gravity + the per-physics-substep venue aero wrench (drag + Magnus,
  ``F = m(-k_d|v|v + k_m omega x v)``, world->body rotated because Isaac Lab 2.1
  ``set_external_force_and_torque`` applies wrenches in the BODY frame — the
  table_tennis_env.py / shadow_ball.py mechanism, re-used verbatim).
* TABLE BOUNCE — CODE-DRIVEN: a descending crossing of ``surface_z + R`` inside the table
  footprint triggers :func:`predict_table_contact` (the fitted angle-dependent tangential-impulse
  contact with the VENUE TABLE params ``contact.table`` of configs/ball_physics_venue.yaml:
  constant e_eff, v_r = 0, n = +z). PhysX NEVER resolves the ball's contacts (see below), so the
  fitted model is the single bounce authority — no restitution double-count.
* RACKET IMPULSE (PHASE B, ``physical_ball_impulse=True``) — CODE-DRIVEN, like the table bounce:
  each physics substep the manager reads the blade pose through the command's PURE FK helper
  (``RacketTargetCommand._racket_fk`` — the same math ``_compute_racket_state`` assigns from,
  returned as LOCALS; blade-center contact-point velocity incl. omega x mount-offset; no fourth
  FK derivation). The command's reward/obs-visible buffers ``racket_pos_w`` / ``racket_quat_w`` /
  ``racket_lin_vel_w`` / ``racket_normal_w`` are NEVER written from the substep path: Isaac Lab
  2.1 runs reward_manager BEFORE command_manager.compute, so a substep rebind would leak
  mid-step-fresh FK into the reward stream and break the metrics-only contract
  (adversarial-review fix; tensor-identity regression-tested). The scan runs
  :func:`blade_disc_contact`:
  a disc of radius ``RACKET_CONTACT_RADIUS`` (0.075 m — hope_planner.constants.racket_radius ==
  table_tennis_env.RACKET_CONTACT_RADIUS) at the blade pose, slab test |d_n| <= R + pad OR a
  blade-plane SIGN CROSSING between substeps (the anti-tunnel test — at 200 Hz physics and up to
  ~10 m/s closing speed the slab alone can be jumped), plus the table_tennis approaching test.
  On detection :func:`racket_impulse` — a PURE DELEGATION to
  ``virtual_ball.predict_paddle_contact`` (the venue e(u_n) paddle model; NO fourth contact
  implementation) — rewrites the ball's velocity+spin via ``write_root_velocity_to_sim`` (WORLD
  frame; the aero wrench path stays body-frame per Isaac Lab 2.1) AND SNAPS the ball to the
  blade-PLANE contact point via ``write_root_pose_to_sim`` (the table-bounce snap mirrored:
  interpolated plane crossing on the crossing branch, normal projection on the slab branch —
  d_n = 0, the vb channel's blade-plane convention, so the return flight and the analytic vb
  rollout launch from one convention and ``pb_virt_phys_gap_m`` measures channel divergence,
  not detection-sampling offset). ONE impulse per swing
  (``_impulse_done`` latch, re-armed only by ``on_resample``). The struck ball's RETURN flight
  is then measured: first descending ``surface+R`` crossing = ``pb_return_land_x/y`` (+ error vs
  the question's latched landing target (the current tuple's ``intended_landing_xy`` for the
  permanent FH/BH physical cohorts and V14 ``vb_target_x/y`` for core), net-plane crossing at
  ``near_x + NET_X`` with clearance ``center >
  surface + NET_HEIGHT + R`` (the same constants as virtual_ball/hope_commands), return-flight
  table bounces in ``pb_return_bounce_count`` (SPLIT from the Phase A pre-strike honesty counter
  ``pb_bounce_count``, which stays ~0-meaningful), and THE
  cross-ruler ``pb_virt_phys_gap_m`` = |physical return landing - the analytic vb landing
  prediction latched for the SAME strike| (virtual channel vs engine channel divergence — the
  instrument's whole point).
* ROBOT PASS-THROUGH — the ball's collider is DISABLED (``collision_enabled=False``), which
  filters ball<->robot AND ball<->table PhysX contacts in one switch. This is deliberate:
  (a) the racket contact is CODE-DRIVEN even in Phase B (the fitted venue paddle model — a PhysX
  robot collision would be an unfitted artifact double-hitting on top of it); (b) the table
  bounce must be code-authoritative anyway (PhysX restitution cannot represent the fitted
  spin-dependent contact). The table USD is visual-only and contains no PhysX collider. Phase B
  COLLISION-FILTER DECISION (recorded): code is the single contact
  authority for ball<->robot AND ball<->table, so the required per-pair filter set is ALL of the
  ball's pairs — which is exactly what the one-switch disabled collider expresses. Isaac Lab 2.1
  spawn cfgs cannot express PhysX filtered pairs (``UsdPhysics.FilteredPairsAPI``) per pair, and
  the only CCD knob is the SCENE-level ``sim.physx.enable_ccd`` (no per-body field in this
  version, per table_tennis_env_cfg) — with every ball pair filtered CCD would arm NOTHING while
  still touching the robot's solver path, so the collider stays OFF and CCD is SKIPPED (loud
  print at init). Anti-tunnel duty is carried by the blade-plane sign-crossing test above and by
  the segment-based table-plane crossing scan. NO ball<->anything PhysX contact is ever enabled
  silently.

PHYSICS BASIS: scripts/isaac_ball_inloop_check.py validated exactly this injection pattern
(batched single view, per-substep body-frame venue aero wrench) with an in-loop result of PhysX
flight matching the venue RK4 reference to a 17 mm SYSTEMATIC landing offset — that number is the
expected floor for ``pb_serve_err_m`` here (reverse-RK4 launch -> forward PhysX-Euler flight).

WHAT THIS IS NOT: a TRUTH INSTRUMENT only. Reward and observation streams are COMPLETELY
untouched even when the flag is on — no reward term, no obs term, no bank-target logic reads any
of this; the analytic virtual ball (:mod:`virtual_ball`) remains the reward machine. The value is
per-strike ground truth: ``pb_serve_err_m`` / ``pb_serve_vel_err`` measure how exactly the engine
delivers the question's (contact point, incoming velocity) at the strike frame, and the
post-strike flight/bounce/landing metrics record what the real ball did.

HONESTY NOTES (read before trusting the numbers):

* Phase A serves ONLY the FINAL BALLISTIC SEGMENT (post-last-bounce): the reverse integration
  TRUNCATES at the table plane (last state strictly above ``surface_z + R + SERVE_PLANE_MARGIN``)
  and returns the per-env ``tts_effective`` it actually covered. Questions whose real history
  includes the incoming table bounce (rising contact velocities — ~11% of the bank — and moderate
  tts generally) launch LATER, ``tts_effective`` before the strike, from ON the incoming
  trajectory, so forward flight for ``tts_effective`` still arrives exactly at the question
  (contact, velocity) — the arrival guarantee the instrument needs. The pre-bounce segment is
  OUT OF SCOPE until the bounce-aware serve (bounce-map inversion — future work). This fix is
  the seed=1 pod-defect root cause: un-truncated back-integration put rising-contact launches
  under/inside the table (pb_serve_err_m 0.58 m); seed=0 had merely been lucky with questions.
* With ``physical_ball_impulse=False`` (default) the strike applies NO impulse (Phase A
  behavior, byte-identical): the ball flies THROUGH the strike point and the robot, descends
  behind it, and its first descending ``surface+R`` crossing is recorded as the landing (same
  plane convention as virtual_ball.coarse_landing / the shadow ball). With the impulse ON, a
  swing whose blade physically contacts the ball BEFORE the exact-strike frame consumes that
  swing's serve measurement (the inbound arrival is no longer measurable — the hit is counted in
  ``pb_hit_count`` instead), and a swing whose blade never meets the ball keeps the Phase A
  pass-through landing path.
* SUBSTEP KNOB (``physical_ball_substep``, default 1 = OFF — decision CLOSED for S1: the venue
  band passed in-loop at 17 mm without it; the knob exists for S3 fast balls): >1 integrates the
  venue flight law with N Euler substeps INSIDE each physics callback and applies the
  velocity-matched average aero force (:func:`substepped_aero_force`). APPROXIMATION, stated
  honestly: only the VELOCITY update is substep-corrected — PhysX still integrates the position
  with one constant acceleration over its own dt (true substepping needs a smaller sim-level
  dt), so per-step position curvature error remains first-order. N=1 reduces EXACTLY to
  ``shadow_ball.venue_aero_force`` (tested), so the default is byte-identical to Phase A.
* The pre-strike arc cannot trigger the table bounce by construction (the bounce fires only on
  DESCENDING in-bounds crossings; the inbound arc's minimum over the table is the contact point
  itself, which sits above ``surface+R``). If an out-of-envelope question ever does descend
  through the plane in-bounds pre-strike, the bounce is applied anyway (physical consistency) and
  ``pb_serve_err_m`` reports the damage honestly. ``pb_bounce_count`` counts ONLY non-RETURN-mode
  bounces so this "~0 by construction" honesty meaning survives Phase B: successful returns
  land and re-bounce on the table by design, and those go to ``pb_return_bounce_count`` instead
  (adversarial-review fix — pre-split, impulse-on runs read pb_bounce_count ~ 1-2x
  pb_return_count and the invariant silently inverted).

This module is importable WITHOUT Isaac (torch-only at top level; sibling modules are loaded by
file path when the package import is unavailable). Pure helpers are unit-tested Isaac-free in
``tests/test_physical_ball_helpers.py``.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import torch

# --------------------------------------------------------------------------------------------- #
# Sibling modules: package import in the training env; file-path load for Isaac-free tests
# (the mdp package __init__ pulls isaaclab, so standalone loading cannot go through it).
# --------------------------------------------------------------------------------------------- #
try:  # pragma: no cover - trivial import plumbing
    from whole_body_tracking.tasks.tracking.mdp import shadow_ball as _sb
    from whole_body_tracking.tasks.tracking.mdp import virtual_ball as _vb
except Exception:  # standalone (tests / scripts without isaaclab on the path)

    def _load_sibling(fname: str, name: str):
        import importlib.util
        import sys

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), fname)
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod  # dataclass resolution needs the module registered during exec
        spec.loader.exec_module(mod)
        return mod

    _sb = _load_sibling("shadow_ball.py", "physical_ball._shadow_ball")
    _vb = _load_sibling("virtual_ball.py", "physical_ball._virtual_ball")

# Serve horizon (s): the ball launches when time_to_strike first drops to/below this. Bounded for
# (a) the same env-footprint reason as shadow_ball.PRESTRIKE_HORIZON_S, (b) the incoming-bounce
# honesty window (module docstring), and (c) reverse-time drag blowup: integrating quadratic drag
# BACKWARD anti-amplifies speed with a finite-time singularity, measured at t* ~ 1.3 s for venue
# contact states (speed ~8-10 m/s at 0.6 s, ~22-25 m/s at 1.0 s, divergent by ~1.3 s) — 0.6 s
# keeps a >2x margin below both the blowup and BACKINT_SPEED_CAP.
SERVE_HORIZON_S = 0.6
# Reverse-integration speed cap (m/s): rows whose backward speed WOULD exceed this stop stepping
# (the crossing step is rejected), so the helper stays finite for any requested tts (up to and
# beyond 1.5 s) instead of hitting the reverse-drag singularity. The cap NEVER engages within
# the venue velocity envelope for t_back <= ~1.0 s (backward speeds reach ~22-32 m/s there —
# tested); capped rows report the shorter ``tts_effective`` they actually integrated, and the
# roundtrip guarantee holds over that span like any truncated row.
BACKINT_SPEED_CAP = 40.0
# Table-plane truncation margin (m): the reverse integration STOPS at the last state strictly
# above z = surface_z + ball_radius + SERVE_PLANE_MARGIN. Root cause of the seed=1 pod defect
# (pb_serve_err_m = 0.58 m / pb_serve_vel_err = 1.19): for rising-contact questions (vz >= 0,
# ~11% of the bank) and moderate tts the pure backward path dips below the table plane — in
# reality that segment is PRE-BOUNCE — so the launch state sat under/inside the table and the
# serve was garbage. Phase A serves only the FINAL ballistic segment; the pre-bounce segment is
# the future bounce-aware serve (module docstring).
SERVE_PLANE_MARGIN = 5e-3
# Reverse-integration step used by the manager at serve time (helper default is 1e-3 for tests).
# RK4 truncation at 5 ms over <= 0.6 s is sub-mm — far below the 17 mm engine-integration floor.
SERVE_BACKINT_H = 5e-3
# Park position (env-local): far below the table, out of sight; rewritten kinematically every
# control step so nothing (gravity, stale forces) can accumulate on a parked ball.
PARK_POS_ENV = (0.0, 0.0, -10.0)
# Post-strike balls below this env-local z are done (landing recorded or hopeless) -> park.
KILL_Z_ENV = -2.0
# Phase B blade footprint: the A3 paddle is ~7.5 cm radius — the SAME constant everywhere
# (hope_planner.constants.racket_radius == table_tennis_env.RACKET_CONTACT_RADIUS == 0.075).
# Redefined here (not imported) only because table_tennis_env pulls isaaclab at import time and
# this module must stay Isaac-free; the value is pinned by test_impulse_detection.
RACKET_CONTACT_RADIUS = 0.075
# Face-slab half-thickness pad beyond the ball radius for the disc contact test (m): absorbs the
# per-substep discretization of a grazing approach. The sign-crossing branch (not this pad) is
# the anti-tunnel guarantee for fast normal closings.
BLADE_CONTACT_PAD = 0.005

_MODE_PARKED = 0   # waiting for the question / for tts to enter the serve horizon
_MODE_INBOUND = 1  # launched; PhysX + aero wrench own the flight; strike frame not yet reached
_MODE_POST = 2     # past the strike frame with no impulse applied; flying until landing/kill
_MODE_RETURN = 3   # Phase B: racket impulse applied; return flight until landing/kill


# --------------------------------------------------------------------------------------------- #
# Pure helpers (torch-only; unit-tested Isaac-free)
# --------------------------------------------------------------------------------------------- #
@dataclass(frozen=True)
class TableContactParams:
    """Venue TABLE contact constants (configs/ball_physics_venue.yaml ``contact.table``) +
    the ball constants the contact math needs. The table is static (v_r = 0, n = +z) and uses a
    CONSTANT e_eff (the F4 velocity-dependent restitution applies to the PADDLE only)."""

    e_eff: float
    a_t: float
    b_t: float
    mu: float
    ball_radius: float
    inertia_coeff: float
    source_path: str


def load_venue_table_params(path: str | None = None) -> TableContactParams:
    """Read the table-contact block from the SAME venue yaml the flight/paddle constants use."""
    import yaml

    path = path or _vb.default_venue_yaml_path()
    with open(path, "r") as fh:
        raw = yaml.safe_load(fh)
    tab = raw["contact"]["table"]
    return TableContactParams(
        e_eff=float(tab["e_eff"]),
        a_t=float(tab["a_t"]),
        b_t=float(tab["b_t"]),
        mu=float(tab["mu_safety"]),
        ball_radius=float(raw["ball"]["radius"]),
        inertia_coeff=float(raw["ball"]["inertia_coeff"]),
        source_path=os.path.abspath(path),
    )


def back_integrate_incoming(
    contact_pos: torch.Tensor,
    incoming_vel: torch.Tensor,
    omega: torch.Tensor,
    tts: torch.Tensor,
    prm,
    h: float = 1e-3,
    surface_z: float = 0.76,
    margin: float = SERVE_PLANE_MARGIN,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Venue-model REVERSE-TIME integration from the contact state, TRUNCATED at the table plane.

    Integrates the fitted flight ODE ``a = g - k_d|v|v + k_m (omega x v)`` backward
    (``virtual_ball.rk4_step`` with a NEGATIVE per-env step) for up to ``tts`` seconds, STOPPING
    per env at the last state strictly above ``z = surface_z + ball_radius + margin`` (and below
    the ``BACKINT_SPEED_CAP`` reverse-drag guard). The returned state is ON the incoming
    trajectory, so forward flight from ``(launch_pos, launch_vel)`` for ``tts_effective`` seconds
    arrives at ``(contact_pos, incoming_vel)`` — roundtrip error is RK4 truncation only (sub-mm
    at h = 1e-3; tested untruncated at tts 0.3/0.6/1.0 s and truncated for rising/long-tts
    cases).

    WHY TRUNCATE (seed=1 pod defect): for rising contact velocities (vz >= 0) and moderate tts
    the pure backward path dips below the table plane — in reality that segment is PRE-BOUNCE —
    and an un-truncated launch sat under/inside the table (pb_serve_err_m 0.58 m). Phase A
    serves only the FINAL BALLISTIC SEGMENT (post-last-bounce); realizing the pre-bounce segment
    (bounce-map inversion) is the future bounce-aware serve. ``tts_effective`` runs ~0.14-0.35 s
    for typical bank contact heights — the serve simply fires later.

    Vectorized over envs with PER-ENV step size: ``n = ceil(max(tts)/h)`` fixed-length loop,
    ``h_i = tts_i / n`` (envs with smaller tts get a smaller, MORE accurate step; ``tts_i = 0``
    rows take identity steps); a row that would step below the plane (or past the speed cap)
    rejects that step and freezes, so ``tts_effective`` is an exact multiple of its ``h_i``.
    Frame-free in xy; ``surface_z`` must be given in the SAME frame as ``contact_pos`` (env-local
    ``vb_table_surface_z`` when positions are env-local, or origin-shifted when world — the
    tracking env grids are z-flat so the manager passes world contact points with the env-local
    plane unchanged). Omega is constant in flight (the fit's assumption).

    Args:
        contact_pos: (N, 3) question contact point.
        incoming_vel: (N, 3) question incoming velocity AT the contact point.
        omega: (N, 3) question incoming spin (rad/s, constant in flight).
        tts: (N,) time to strike in seconds (clamped at 0 from below).
        prm: ``virtual_ball.VirtualBallParams`` (venue flight constants).
        h: nominal reverse step size (s).
        surface_z: table surface height in the frame of ``contact_pos``.
        margin: extra clearance above ``surface_z + ball_radius`` where truncation stops.

    Returns:
        ``(launch_pos, launch_vel, tts_effective)``: (N, 3), (N, 3), (N,). ``tts_effective ==
        tts`` where nothing truncated; smaller where the plane (or the speed cap) cut the span.
    """
    t_back = tts.clamp(min=0.0)
    t_max = float(t_back.max().item()) if t_back.numel() else 0.0
    if t_max <= 0.0:
        return contact_pos.clone(), incoming_vel.clone(), torch.zeros_like(t_back)
    z_min = float(surface_z) + float(prm.ball_radius) + float(margin)
    n_steps = max(1, int(math.ceil(t_max / float(h))))
    h_i = (t_back / float(n_steps)).unsqueeze(-1)  # (N, 1), broadcasts through rk4_step
    p, v = contact_pos, incoming_vel
    t_eff = torch.zeros_like(t_back)
    alive = torch.ones_like(t_back, dtype=torch.bool)
    for _ in range(n_steps):
        p_new, v_new = _vb.rk4_step(p, v, omega, -h_i, prm)
        # Accept the step only where the row is still integrating AND the new state stays
        # strictly above the truncation plane AND below the reverse-drag speed cap; a rejected
        # step freezes the row at the last valid state (its candidate recomputes identically and
        # keeps being rejected — no NaN path).
        ok = (
            alive
            & (p_new[:, 2] > z_min)
            & (torch.linalg.norm(v_new, dim=-1) < BACKINT_SPEED_CAP)
        )
        okc = ok.unsqueeze(-1)
        p = torch.where(okc, p_new, p)
        v = torch.where(okc, v_new, v)
        t_eff = t_eff + h_i.squeeze(-1) * ok
        alive = ok
    return p, v, t_eff


def predict_table_contact(
    v_minus: torch.Tensor,
    omega_minus: torch.Tensor,
    tp: TableContactParams,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Fitted table bounce: the spin-equation contact with the venue TABLE params.

    Same angle-dependent tangential-impulse math as ``virtual_ball.predict_paddle_contact`` /
    ``ball_physics_fit/contact_model.predict_contact`` specialized to the static table:
    ``v_r = 0``, ``n = +z`` (the bounce caller guarantees a descending ball, so the oriented
    normal is +z by construction), CONSTANT ``e_eff``::

        u    = v- + w- x (-R n)
        s    = clip((a_t + b_t cos(theta)) |u_t|, 0, mu (1+e) |u_n|)
        dv_t = -s unit(u_t);  dv_n = -(1+e) u_n n;  dw = -(1/(cR)) n x dv_t

    Inputs (N, 3); returns ``(v_plus, omega_plus)``.
    """
    eps = 1e-9
    R = tp.ball_radius
    c = tp.inertia_coeff
    n = torch.zeros_like(v_minus)
    n[:, 2] = 1.0
    r = -R * n

    u = v_minus + torch.cross(omega_minus, r, dim=-1)
    u_n_signed = torch.sum(u * n, dim=-1, keepdim=True)          # (N, 1), < 0 for a descending ball
    u_t_vec = u - u_n_signed * n
    u_t_mag = torch.linalg.norm(u_t_vec, dim=-1, keepdim=True)
    u_n_abs = torch.abs(u_n_signed)

    cos_theta = u_n_abs / (torch.hypot(u_t_mag, u_n_signed) + eps)
    raw = (tp.a_t + tp.b_t * cos_theta) * u_t_mag
    cap = tp.mu * (1.0 + tp.e_eff) * u_n_abs
    s = torch.clamp(raw, min=0.0).minimum(cap)

    safe_dir = u_t_vec / (u_t_mag + eps)
    delta_v_t = torch.where(u_t_mag > eps, -s * safe_dir, torch.zeros_like(u_t_vec))
    delta_v_n = -(1.0 + tp.e_eff) * u_n_signed * n
    delta_omega = -(1.0 / (c * R)) * torch.cross(n, delta_v_t, dim=-1)

    return v_minus + delta_v_n + delta_v_t, omega_minus + delta_omega


def schedule_serves(
    parked: torch.Tensor,
    tts: torch.Tensor,
    horizon_s: float = SERVE_HORIZON_S,
    min_tts_s: float = 0.0,
) -> torch.Tensor:
    """Which parked envs launch their ball THIS control step.

    A parked env serves the first step its ``time_to_strike`` enters ``(min_tts_s, horizon_s]``:
    beyond the horizon it keeps waiting (parked far below the table); at/below ``min_tts_s``
    (the manager passes one control step) there is no physics window left for the ball to fly, so
    it never serves — the strike is then counted as ``pb_missed_serve`` (happens when a resample
    lands inside the last control step before the strike, or the reference clock jumps).
    """
    return parked & (tts <= float(horizon_s)) & (tts > float(min_tts_s))


def table_bounds_mask(
    xy: torch.Tensor, near_x: float, table_len: float, half_w: float
) -> torch.Tensor:
    """Env-local footprint test for the code-driven bounce: x in [near, near+len], |y| <= half_w."""
    return (
        (xy[:, 0] >= float(near_x))
        & (xy[:, 0] <= float(near_x) + float(table_len))
        & (xy[:, 1].abs() <= float(half_w))
    )


# --------------------------------------------------------------------------------------------- #
# Phase B pure helpers (torch-only; unit-tested Isaac-free)
# --------------------------------------------------------------------------------------------- #
def racket_impulse(
    v_ball: torch.Tensor,
    v_blade: torch.Tensor,
    n_blade: torch.Tensor,
    omega_ball: torch.Tensor,
    prm,
) -> tuple[torch.Tensor, torch.Tensor]:
    """The in-engine racket impulse — a PURE DELEGATION to the venue paddle model.

    This function exists so the manager has exactly one named seam for the impulse and the test
    suite can assert it IS ``virtual_ball.predict_paddle_contact`` (e(u_n) restitution, oriented
    normal, spin equation) on random states — the guard against a fourth contact implementation
    ever drifting in. ``v_blade`` is the blade-center contact-point velocity
    (``racket_lin_vel_w``, which already includes omega x mount-offset), the SAME ``v_r`` the
    reward path feeds the model, so the virtual and engine channels share one contact input
    convention and ``pb_virt_phys_gap_m`` measures channel divergence, not convention skew.
    """
    return _vb.predict_paddle_contact(v_ball, v_blade, n_blade, omega_ball, prm)


def blade_disc_contact(
    ball_pos: torch.Tensor,
    ball_vel: torch.Tensor,
    blade_pos: torch.Tensor,
    blade_vel: torch.Tensor,
    blade_normal: torch.Tensor,
    prev_d_n: torch.Tensor,
    prev_valid: torch.Tensor,
    racket_radius: float = RACKET_CONTACT_RADIUS,
    ball_radius: float = 0.02,
    pad: float = BLADE_CONTACT_PAD,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Blade-disc contact test at one sample (mirrors table_tennis_env._handle_paddle, hardened).

    Geometry: ``d = ball - blade_center`` decomposed along the face normal (``d_n`` signed,
    ``d_t`` tangential). A contact fires when the ball center is over the blade disc
    (``d_t <= racket_radius``) AND either

    * SLAB — ``|d_n| <= ball_radius + pad`` while APPROACHING (``(v_ball - v_blade) . d < 0``,
      the sign-free table_tennis test; predict_paddle_contact's orient_normal handles which face
      is struck), or
    * CROSSING — ``d_n`` changed SIGN since the previous sample (``prev_valid`` rows only): the
      ball passed through the blade plane within one physics step. This is the anti-tunnel
      branch (at 200 Hz physics a ~10 m/s closing speed moves 5 cm per step, > the slab) and
      deliberately does NOT require the approaching test — after tunnelling the relative
      velocity already points away.

    Returns ``(hit, d_n)``; the caller stores ``d_n`` as the next step's ``prev_d_n``.
    """
    n_hat = blade_normal / (torch.linalg.norm(blade_normal, dim=-1, keepdim=True) + 1e-9)
    d = ball_pos - blade_pos
    d_n = torch.sum(d * n_hat, dim=-1)
    d_t = torch.linalg.norm(d - d_n.unsqueeze(-1) * n_hat, dim=-1)
    within = d_t <= float(racket_radius)
    slab = d_n.abs() <= (float(ball_radius) + float(pad))
    approaching = torch.sum((ball_vel - blade_vel) * d, dim=-1) < 0.0
    crossed = prev_valid & ((d_n * prev_d_n) < 0.0)
    return within & ((slab & approaching) | crossed), d_n


def net_plane_crossing(
    prev_pos: torch.Tensor, new_pos: torch.Tensor, net_x: float
) -> tuple[torch.Tensor, torch.Tensor]:
    """First +x crossing of the net plane between two samples: ``(crossed, z_at_crossing)``.

    Same linear-interpolation extraction as ``virtual_ball.coarse_landing``'s net branch, so the
    physical and analytic net-clearance numbers share one convention. Positions in any
    translation-consistent frame (the manager uses env-local; ``net_x = near_x + NET_X``).
    """
    crossed = (prev_pos[:, 0] < float(net_x)) & (new_pos[:, 0] >= float(net_x))
    denom = (new_pos[:, 0] - prev_pos[:, 0]).clamp(min=1e-9)
    f = ((float(net_x) - prev_pos[:, 0]) / denom).clamp(0.0, 1.0)
    z_at = prev_pos[:, 2] + (new_pos[:, 2] - prev_pos[:, 2]) * f
    return crossed, z_at


def substepped_aero_force(
    lin_vel_w: torch.Tensor,
    ang_vel_w: torch.Tensor,
    mass: float,
    prm,
    dt: float,
    n_sub: int,
    speed_clip: float = 50.0,
) -> torch.Tensor:
    """Velocity-matched average aero force over one physics step, N internal Euler substeps.

    Integrates the venue flight law ``a = g + aero(v)`` (omega constant) for ``n_sub`` Euler
    substeps of ``dt / n_sub`` and returns the ONE constant world-frame force
    ``F = m ((v_end - v_start)/dt - g)`` that makes PhysX's own velocity update land exactly on
    the substepped ``v_end``. APPROXIMATION (documented, honest): only the velocity update is
    corrected — PhysX still steps the POSITION with this single constant acceleration, so the
    within-step trajectory curvature error stays first-order; true substepping needs a smaller
    sim-level dt. ``n_sub=1`` reduces EXACTLY to ``shadow_ball.venue_aero_force`` (one Euler
    step: ``F = m * aero(v_start)``) — tested — so the default knob is byte-identical.
    """
    n_sub = max(1, int(n_sub))
    h = float(dt) / n_sub
    v = lin_vel_w
    for _ in range(n_sub):
        a = _sb.venue_aero_force(v, ang_vel_w, 1.0, prm.k_d, prm.k_m, speed_clip)  # accel (m=1)
        a = a.clone()
        a[:, 2] -= prm.g
        v = v + h * a
    dv_dt = (v - lin_vel_w) / float(dt)
    dv_dt = dv_dt.clone()
    dv_dt[:, 2] += prm.g
    return float(mass) * dv_dt


# --------------------------------------------------------------------------------------------- #
# Manager (owned/called by RacketTargetCommand when cfg.physical_ball is on)
# --------------------------------------------------------------------------------------------- #
class PhysicalBallManager:
    """Drives the per-env physical ball through PARKED -> INBOUND -> POST/RETURN. Metrics only.

    SEAM (deliberate, single integration surface): all control-rate work hooks into
    ``RacketTargetCommand`` — ``update(exact_strike)`` once per control step from
    ``_update_metrics`` (after ``_vb_evaluate``), ``on_resample(env_ids)`` from
    ``_resample_command``; the per-substep aero wrench + bounce/landing detection run in a
    ``sim.add_physics_callback`` (the table_tennis_env.py mechanism). Chosen over an interval
    event term (no access to the per-swing resample/tts/question stream without cross-manager
    coupling) and over a scene-entity update (no view of command state at all); the shadow-ball
    driver already proved this seam mech-clean, so both measurement channels share one shape.
    """

    def __init__(self, command, env):
        self._cmd = command
        self._env = env
        self.device = command.device
        n = command.num_envs

        try:
            self._ball = env.scene["pb_ball"]
        except KeyError as exc:
            raise KeyError(
                "PhysicalBallManager: scene entity 'pb_ball' not found. physical_ball=True "
                "requires the scene attachment from hope_env_cfg.attach_physical_ball_scene "
                "(run automatically by HOPEPingPongAgibotA3EnvCfg.__post_init__ or the train.py "
                "task.physical_ball override translation)."
            ) from exc

        # Venue constants: flight via the same loader as the reward path; table contact + mass
        # from the same YAML (single source of truth).
        import yaml as _yaml

        self._prm = _vb.load_venue_params()
        self._tp = load_venue_table_params(self._prm.source_path)
        with open(self._prm.source_path, "r") as fh:
            self._mass = float(_yaml.safe_load(fh)["ball"]["mass"])

        # Virtual-table landmarks (env-local), same convention as the vb reward path. geometry.py
        # is pure python; fall back to a file-path load when the package import is unavailable
        # (Isaac-free harness tests drive the real manager through mocks).
        try:
            from whole_body_tracking.tasks.table_tennis import geometry as _tt_geom
        except Exception:
            import importlib.util as _ilu
            import sys as _sys

            _geo_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "..", "..", "table_tennis", "geometry.py"
            )
            _spec = _ilu.spec_from_file_location("physical_ball._tt_geometry", _geo_path)
            _tt_geom = _ilu.module_from_spec(_spec)
            _sys.modules["physical_ball._tt_geometry"] = _tt_geom  # dataclass resolution needs this
            _spec.loader.exec_module(_tt_geom)

        self._near_x = float(command.cfg.vb_table_near_x)
        self._table_len = float(_tt_geom.TABLE_LENGTH)
        self._half_w = float(_tt_geom.TABLE_WIDTH) / 2.0
        self._z_thr = float(command.cfg.vb_table_surface_z) + float(self._prm.ball_radius)
        # Net landmarks (env-local) — the SAME constants as hope_commands._vb_net_x /
        # _vb_net_top_z + ball radius: net plane at near_x + NET_X (1.37), clearance when the
        # ball CENTER passes above surface + NET_HEIGHT (0.1525) + R.
        self._net_x_env = self._near_x + float(_tt_geom.NET_X)
        self._net_clear_z = (
            float(command.cfg.vb_table_surface_z) + float(_tt_geom.NET_HEIGHT) + float(self._prm.ball_radius)
        )

        # --- Phase B knobs (default OFF = Phase A byte-identical) --------------------------- #
        self._impulse_on = bool(getattr(command.cfg, "physical_ball_impulse", False))
        self._substep = int(getattr(command.cfg, "physical_ball_substep", 1))
        if self._substep < 1:
            raise ValueError(
                f"physical_ball_substep must be >= 1 (1 = off), got {self._substep}."
            )
        # Per-question return target. Core questions use the V14 target; physical-bank questions
        # latch their own intended_landing_xy in on_resample(). This tensor is telemetry-only.
        self._default_target_xy = torch.tensor(
            [float(command.cfg.vb_target_x), float(command.cfg.vb_target_y)],
            device=self.device,
        )
        self._ret_target_xy = self._default_target_xy.repeat(n, 1)
        # 0=V14 core, 1=FH physical tuple, 2=BH physical tuple. Latched with the question so a
        # later command resample cannot relabel an in-flight return.
        self._question_cohort = torch.zeros(
            n, dtype=torch.long, device=self.device
        )
        self._cohort_names = ("core", "fh_physical", "bh_physical")

        # Lifecycle + event buffers.
        self._mode = torch.full((n,), _MODE_PARKED, dtype=torch.long, device=self.device)
        self._landed = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._land_new = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._land_xy = torch.zeros(n, 2, device=self.device)
        self._bounce_new = torch.zeros(n, dtype=torch.bool, device=self.device)
        # Truncation latch: set on any candidate step whose serve is delayed by the plane
        # truncation; COUNTED into pb_serve_truncated_count exactly once, at CONSUMPTION (the
        # serve, or the strike if it never served), where it is also cleared — and ONLY there.
        # Deliberately NOT cleared in on_resample: upstream _resample_command can repeat within
        # one physical wait (motion.just_resampled stays latched across steps at low env counts
        # — the seed=1 exposing config), and a resample-cleared latch either re-counts per
        # candidate step (counting while waiting: the observed 1945 counts vs 110 serves) or
        # never counts at all (counting at serve: the final repeat's fresh discovery serves
        # un-delayed). Consumption events are the only cadence-invariant swing boundary; a latch
        # carried from an aborted wait into the next swing's consumption keeps the AGGREGATE
        # honest (one count per physical wait that ever hit truncation).
        self._trunc_flag = torch.zeros(n, dtype=torch.bool, device=self.device)
        # Per-swing tts_effective cache: the final-ballistic-segment length is a trajectory
        # property fixed when the question is sampled, so it is DISCOVERED once (first candidate
        # step) and cached; waiting steps only compare tts against it instead of re-running the
        # full reverse integration every step (the wasted-compute half of the same defect).
        self._teff_cache = torch.zeros(n, device=self.device)
        self._teff_valid = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._prev_valid = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._prev_pos_env = torch.zeros(n, 3, device=self.device)
        # --- Phase B state (allocated unconditionally — tiny; all logic gated on _impulse_on).
        # One impulse per swing: latched at the hit, re-armed only by on_resample.
        self._impulse_done = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._hit_new = torch.zeros(n, dtype=torch.bool, device=self.device)
        # Blade-plane signed distance at the previous detection sample (the sign-crossing
        # anti-tunnel branch of blade_disc_contact); invalidated on serve/park/resample.
        self._prev_dn = torch.zeros(n, device=self.device)
        self._prev_dn_valid = torch.zeros(n, dtype=torch.bool, device=self.device)
        # Return-flight event buffers: landing + net crossing (z at the net plane, latched once
        # per swing) + the analytic vb landing prediction latched at the SAME strike for the
        # pb_virt_phys_gap_m cross-ruler (the shadow-driver snapshot pattern: vb_landing_xy is
        # clobbered batch-wide on every strike-carrying step).
        self._ret_land_new = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._ret_land_xy = torch.zeros(n, 2, device=self.device)
        # Return-flight bounce latch — SPLIT from _bounce_new (mode-gated at the detector) so
        # pb_bounce_count keeps its Phase A pre-strike-honesty meaning (~0 by construction)
        # while returns legitimately bounce on the opponent half.
        self._ret_bounce_new = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._net_crossed = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._net_z = torch.zeros(n, device=self.device)
        self._pred_xy = torch.zeros(n, 2, device=self.device)
        self._pred_valid = torch.zeros(n, dtype=torch.bool, device=self.device)
        # BankExam Phase-B truth publication. The command may resample at clip completion before
        # the evaluator regains control, so publish the just-ended swing into held buffers before
        # clearing the live latches in on_resample(). No reward/observation reads these fields.
        self._truth_started = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_exam_active = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_attempt_token = torch.full(
            (n,), -1, dtype=torch.long, device=self.device
        )
        self._truth_served = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_exact_seen = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_published = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_published_served = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_published_exact_seen = torch.zeros(
            n, dtype=torch.bool, device=self.device
        )
        self._truth_available = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_contacted = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_net_clear = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_landed_ok = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_returned = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._truth_landing_xy = torch.zeros(n, 2, device=self.device)
        # Host-side activity counters for the physics-callback hot path: refreshed once per
        # control step in update() (arming/activation happen ONLY there — serve), so between
        # control steps they can only OVER-estimate (callback hits/landings merely disarm) and a
        # counter-driven skip is always correct. Replaces the per-substep bool(mask.any())
        # host-device syncs that serialized the GPU pipeline (~12 extra syncs per control step
        # with impulse on — adversarial-review perf finding).
        self._active_host = 0
        self._armed_host = 0
        # Reusable wrench buffers (num_envs, 1 body, 3), zeroed like table_tennis_env.py.
        self._force_b = torch.zeros(n, 1, 3, device=self.device)
        self._torque_b = torch.zeros(n, 1, 3, device=self.device)
        self._identity_quat = torch.zeros(n, 4, device=self.device)
        self._identity_quat[:, 0] = 1.0
        self._park_pos_env = torch.tensor(PARK_POS_ENV, device=self.device).expand(n, 3)

        # Cumulative counters + sample-weighted EMAs (vb-metric discipline: decay only on
        # event-carrying steps — exact at large env counts, slightly stale at small).
        self._serve_count = 0.0
        self._meas_count = 0.0
        self._missed_serve_count = 0.0
        self._trunc_count = 0.0
        self._bounce_count = 0.0
        self._land_count = 0.0
        self._land_on_table_count = 0.0
        self._serve_err_acc = 0.0
        self._serve_vel_err_acc = 0.0
        self._serve_n_acc = 0.0
        # Phase B counters/EMAs (only logged when the impulse is on — off-mode metric keys stay
        # byte-identical to Phase A).
        self._hit_count = 0.0
        self._ret_land_count = 0.0
        self._ret_bounce_count = 0.0
        self._net_clear_count = 0.0
        self._ret_err_acc = 0.0
        self._ret_err_n_acc = 0.0
        self._gap_acc = 0.0
        self._gap_n_acc = 0.0
        self._cohort_stats = {
            name: torch.zeros(3, device=self.device)
            for name in (
                "question",
                "hit",
                "return",
                "bounce",
                "net_clear",
                "opponent_land",
                "land_err_sum",
                "land_err_n",
                "gap_sum",
                "gap_n",
            )
        }
        m = command.metrics
        m["pb_serve_err_m"] = torch.zeros(n, device=self.device)
        m["pb_serve_vel_err"] = torch.zeros(n, device=self.device)
        m["pb_serve_count"] = torch.zeros(n, device=self.device)
        m["pb_strike_meas_count"] = torch.zeros(n, device=self.device)
        m["pb_missed_serve_count"] = torch.zeros(n, device=self.device)
        m["pb_serve_truncated_count"] = torch.zeros(n, device=self.device)
        m["pb_bounce_count"] = torch.zeros(n, device=self.device)
        m["pb_land_count"] = torch.zeros(n, device=self.device)
        m["pb_land_on_table_count"] = torch.zeros(n, device=self.device)
        m["pb_land_x"] = torch.zeros(n, device=self.device)
        m["pb_land_y"] = torch.zeros(n, device=self.device)
        if self._impulse_on:
            m["pb_hit_count"] = torch.zeros(n, device=self.device)
            m["pb_return_count"] = torch.zeros(n, device=self.device)
            m["pb_return_bounce_count"] = torch.zeros(n, device=self.device)
            m["pb_return_net_clear_count"] = torch.zeros(n, device=self.device)
            m["pb_return_net_clear_rate"] = torch.zeros(n, device=self.device)
            m["pb_return_land_x"] = torch.zeros(n, device=self.device)
            m["pb_return_land_y"] = torch.zeros(n, device=self.device)
            m["pb_return_land_err_m"] = torch.zeros(n, device=self.device)
            m["pb_virt_phys_gap_m"] = torch.zeros(n, device=self.device)
            m["pb_question_cohort"] = torch.zeros(n, device=self.device)
            for name in self._cohort_names:
                for suffix in (
                    "question_count",
                    "hit_count",
                    "return_count",
                    "bounce_count",
                    "net_clear_rate",
                    "opponent_land_rate",
                    "landing_error_m",
                    "analytic_physical_gap_m",
                ):
                    m[f"pb_{name}_{suffix}"] = torch.zeros(
                        n, device=self.device
                    )

        # Per-substep aero + bounce/landing via the table_tennis_env.py physics callback. The
        # callback is part of the truth-instrument contract; starting without it would publish
        # plausible-looking but wrong control-rate telemetry, so registration fails closed.
        self._cb_active = False
        try:
            env.sim.add_physics_callback("hope_physical_ball", self._on_physics_step)
            self._cb_active = True
        except Exception as exc:  # pragma: no cover - environment-dependent
            raise RuntimeError(
                "PhysicalBallManager requires the physics callback; refusing to start with "
                "degraded telemetry"
            ) from exc
        print(
            f"[PhysicalBallManager] PHYSICAL ball ON (truth instrument, metrics-only): "
            f"R={self._prm.ball_radius} m, mass={self._mass} kg, k_d={self._prm.k_d}, "
            f"k_m={self._prm.k_m}, table e_eff={self._tp.e_eff} a_t={self._tp.a_t} "
            f"mu={self._tp.mu} (code-driven bounce), serve horizon={SERVE_HORIZON_S}s, "
            f"bounce plane z={self._z_thr:.4f} env-local, x in "
            f"[{self._near_x:.2f}, {self._near_x + self._table_len:.2f}], |y|<={self._half_w:.3f}. "
            + (
                "Racket impulse = Phase B ON (code-driven venue paddle model)."
                if self._impulse_on
                else "Racket impulse OFF (physical_ball_impulse=False: ball passes through the robot)."
            ),
            flush=True,
        )
        if self._impulse_on:
            # COLLISION FILTER / CCD DECISION (loud, per the Phase B board claim): code is the
            # single contact authority for ball<->robot AND ball<->table, so the required
            # per-pair filter set is ALL of the ball's pairs — expressed by the one-switch
            # disabled collider (attach_physical_ball_scene). Isaac Lab 2.1 spawn cfgs cannot
            # express UsdPhysics.FilteredPairsAPI per pair, and the ONLY CCD knob is scene-level
            # sim.physx.enable_ccd (no per-body field in this version, see table_tennis_env_cfg)
            # — with every ball pair filtered it would arm NOTHING while touching the robot's
            # solver path. Fallback taken: collider OFF, CCD SKIPPED; anti-tunnel duty = the
            # blade-plane sign-crossing detection + the segment-based table-plane scan.
            print(
                "[PhysicalBallManager] Phase B impulse: blade disc R="
                f"{RACKET_CONTACT_RADIUS} m at the command FK blade pose, one impulse/swing, "
                f"outgoing state = virtual_ball.predict_paddle_contact (venue e(u_n) model, "
                f"delegation-tested); return metrics use each question's latched landing target, "
                f"net plane x={self._net_x_env:.2f}, "
                f"clear z>{self._net_clear_z:.4f}. COLLISION AUTHORITY = CODE for ball<->robot "
                "AND ball<->table: ball collider stays DISABLED (Isaac Lab 2.1 cfg has no "
                "per-pair FilteredPairsAPI surface; scene-level physx.enable_ccd is the only CCD "
                "knob and arms nothing with all ball pairs filtered) -> per-pair filter = the "
                "one-switch disabled collider, CCD SKIPPED (documented fallback; no ball<->anything "
                "PhysX contact enabled).",
                flush=True,
            )
        if self._substep > 1:
            print(
                f"[PhysicalBallManager] ball-substep mechanism ON: n_sub={self._substep} — the "
                "aero wrench becomes the velocity-matched average force of an internally "
                "Euler-substepped venue flight law per physics step (substepped_aero_force). "
                "APPROXIMATION: velocity-level only; PhysX still steps the position with one "
                "constant acceleration per dt (true substepping needs a smaller sim-level dt). "
                "S1 default is 1 (OFF) — the venue band passed in-loop at 17 mm without it.",
                flush=True,
            )

    # ------------------------------------------------------------------ #
    # control-rate hooks (called from RacketTargetCommand)
    # ------------------------------------------------------------------ #
    def on_resample(self, env_ids) -> None:
        """New question for these envs (reset or clip wrap): park until tts enters the horizon.

        CONSUME-BEFORE-CLEAR (adversarial-review fix): on the TRUE-RESET path this runs from
        ``CommandTerm.reset -> _resample_command`` BEFORE this step's ``update()`` /
        ``_consume_events`` (Isaac Lab 2.1: ``_reset_idx`` precedes ``command_manager.compute``),
        so a hit / landing / bounce the physics callback latched THIS control step — an impulse
        already applied in-engine — would otherwise be cleared below without ever reaching
        ``pb_hit_count`` / ``pb_return_count`` (+ their landing/net/gap bookkeeping): the
        counters would strictly undercount engine-applied events. Fold all pending events first
        (batch-wide and idempotent — flags are zeroed on consumption; the WRAP path already
        consumed this step's events in ``update()``, making this a no-op there).
        """
        self._consume_events()
        ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        if self._impulse_on:
            self._publish_cross_engine_truth(ids)
            selected = self._cmd._venue_tuple_selected[ids]
            fixed_side = getattr(self._cmd, "_venue_tuple_cohort", None)
            if torch.is_tensor(fixed_side):
                side = fixed_side[ids].clone()
            else:
                side = torch.full_like(ids, -1)
            motion = self._cmd._motion()
            live_clip = (
                motion.clip_id[ids]
                if getattr(motion, "_multiseg", False)
                else torch.zeros_like(ids)
            )
            side = torch.where(selected & (side < 0), live_clip, side)
            cohort = torch.where(selected, side + 1, torch.zeros_like(side))
            self._question_cohort[ids] = cohort
            target = self._default_target_xy.expand(len(ids), 2).clone()
            target[selected] = self._cmd._venue_intended_landing_xy[
                ids[selected]
            ]
            self._ret_target_xy[ids] = target
            self._cmd.metrics["pb_question_cohort"][ids] = cohort.float()
            self._cohort_stats["question"] += torch.bincount(
                cohort, minlength=3
            ).float()
        self._mode[ids] = _MODE_PARKED
        self._landed[ids] = False
        self._land_new[ids] = False
        self._bounce_new[ids] = False
        # NOTE: _trunc_flag is NOT cleared here — it is consumed (counted + cleared) at the
        # serve/strike only, so pb_serve_truncated_count stays exactly-once-per-wait even when
        # upstream resamples repeat within one wait (see the latch comment in __init__).
        self._teff_valid[ids] = False  # new question -> new trajectory -> re-discover its segment
        self._prev_valid[ids] = False
        # Phase B: re-arm the one-impulse-per-swing latch and clear the swing's return/net/pred
        # state. A return flight still in the air is cut short (no landing recorded for that
        # swing) — the shadow-driver convention. (After the consume above these per-id clears
        # are belt-and-suspenders: all event flags are already zero batch-wide.)
        self._impulse_done[ids] = False
        self._hit_new[ids] = False
        self._prev_dn_valid[ids] = False
        self._ret_land_new[ids] = False
        self._ret_bounce_new[ids] = False
        self._net_crossed[ids] = False
        self._pred_valid[ids] = False
        # BankExam attempt ownership is armed only by begin_external_exam_attempt(), after the
        # immutable external motion/question pair has been installed. Reset-time/train-question
        # resamples must never create a held physical outcome for the later exam row.

    def begin_external_exam_attempt(self, env_ids, attempt_tokens) -> None:
        """Arm one evaluator-owned question generation after its atomic install.

        The evaluator calls this exactly once after reset plus external motion/question install.
        It clears any reset-time/train-question publication and binds the held truth to the
        immutable schedule index. Phase B deliberately supports one external question per env;
        continuous T1 training never calls this seam and cannot consume this score lane.
        """

        if self.cross_engine_truth_capability != (
            "physical_paddle_contact_and_post_contact_flight_v1"
        ):
            raise RuntimeError(
                "external physical truth requires the full physics-substep Phase-B capability"
            )
        ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device).reshape(-1)
        tokens = torch.as_tensor(
            attempt_tokens, dtype=torch.long, device=self.device
        ).reshape(-1)
        if len(ids) == 0 or len(ids) != len(tokens):
            raise ValueError("external physical truth requires equal non-empty ids/tokens")
        if len(torch.unique(ids)) != len(ids) or torch.any(ids < 0) or torch.any(
            ids >= self._cmd.num_envs
        ):
            raise ValueError("external physical truth env ids must be unique and in range")
        if len(torch.unique(tokens)) != len(tokens) or torch.any(tokens < 0):
            raise ValueError("external physical truth attempt tokens must be unique/non-negative")
        if bool(self._truth_exam_active[ids].any()):
            raise RuntimeError(
                "external physical truth supports exactly one begin generation per env"
            )
        if bool((self._mode[ids] != _MODE_PARKED).any()):
            raise RuntimeError("external physical truth must begin with every ball parked")

        self._truth_exam_active[ids] = True
        self._truth_attempt_token[ids] = tokens
        self._truth_started[ids] = True
        self._truth_served[ids] = False
        self._truth_exact_seen[ids] = False
        self._truth_published[ids] = False
        self._truth_published_served[ids] = False
        self._truth_published_exact_seen[ids] = False
        self._truth_available[ids] = False
        self._truth_contacted[ids] = False
        self._truth_net_clear[ids] = False
        self._truth_landed_ok[ids] = False
        self._truth_returned[ids] = False
        self._truth_landing_xy[ids] = 0.0
        # Live Phase-B latches should already be clear from reset/on_resample; clear them again at
        # this evaluator-owned generation boundary so a stale train question cannot leak in.
        self._impulse_done[ids] = False
        self._landed[ids] = False
        self._net_crossed[ids] = False
        self._net_z[ids] = 0.0
        self._ret_land_xy[ids] = 0.0
        self._hit_new[ids] = False
        self._ret_land_new[ids] = False
        self._ret_bounce_new[ids] = False
        self._pred_valid[ids] = False
        self._prev_dn_valid[ids] = False

    @property
    def cross_engine_truth_capability(self) -> str:
        if not self._impulse_on:
            return "incoming_flight_only_no_paddle_contact_phase_a"
        if not self._cb_active:
            return "phase_b_invalid_missing_physics_callback"
        return "physical_paddle_contact_and_post_contact_flight_v1"

    @property
    def cross_engine_truth_metadata(self) -> dict:
        """Explicit runtime capability record consumed by the evaluator-owned BankExam.

        Construction fails when the physics callback cannot be registered, so a live manager
        cannot silently fall back to control-rate-only telemetry.  Nothing in rewards,
        observations, or the legacy virtual score reads this record.
        """

        capability = self.cross_engine_truth_capability
        return {
            "available": capability == "physical_paddle_contact_and_post_contact_flight_v1",
            "capability": capability,
            "physics_callback_active": bool(self._cb_active),
            "racket_impulse_enabled": bool(self._impulse_on),
            "aero_substep": int(self._substep),
            "contact_authority": "code_driven_blade_disc_and_venue_paddle_impulse",
            "post_contact_rollout": (
                "physx_gravity_plus_deterministic_venue_aero_and_code_table_bounce"
            ),
            "collision_authority": "code_only_ball_collider_disabled",
            "racket_contact_radius_m": float(RACKET_CONTACT_RADIUS),
            "ball_radius_m": float(self._prm.ball_radius),
            "reason": (
                "full physics-substep Phase-B truth instrument active"
                if capability == "physical_paddle_contact_and_post_contact_flight_v1"
                else (
                    "invalid manager state: required physics callback is unavailable"
                    if self._impulse_on
                    else "Phase B racket impulse is disabled"
                )
            ),
        }

    def _publish_cross_engine_truth(self, ids: torch.Tensor) -> None:
        """Latch the ended swing before command resampling clears its live Phase-B state.

        First-unconsumed publication wins.  Some low-env command paths can repeat resampling
        before the evaluator regains control; allowing a second empty resample to overwrite the
        just-ended contacted flight would silently turn a physical return into a no-contact row.
        Formal BankExam owns one question per env, so retaining the first held result is exact;
        non-formal training never reads these buffers.
        """

        valid = (
            self._truth_exam_active[ids]
            & self._truth_started[ids]
            & ~self._truth_published[ids]
        )
        contacted = self._impulse_done[ids]
        landed = contacted & self._landed[ids]
        net_clear = landed & self._net_crossed[ids] & (self._net_z[ids] > self._net_clear_z)
        xy = self._ret_land_xy[ids]
        landed_ok = (
            landed
            & (xy[:, 0] > self._net_x_env)
            & (xy[:, 0] <= self._near_x + self._table_len)
            & (xy[:, 1].abs() <= self._half_w)
        )
        available = (
            valid
            & self._truth_served[ids]
            & self._truth_exact_seen[ids]
            & ((~contacted) | landed)
        )
        self._truth_published[ids] |= valid
        self._truth_published_served[ids] = torch.where(
            valid, self._truth_served[ids], self._truth_published_served[ids]
        )
        self._truth_published_exact_seen[ids] = torch.where(
            valid, self._truth_exact_seen[ids], self._truth_published_exact_seen[ids]
        )
        self._truth_available[ids] = torch.where(valid, available, self._truth_available[ids])
        self._truth_contacted[ids] = torch.where(valid, contacted, self._truth_contacted[ids])
        self._truth_net_clear[ids] = torch.where(valid, net_clear, self._truth_net_clear[ids])
        self._truth_landed_ok[ids] = torch.where(valid, landed_ok, self._truth_landed_ok[ids])
        self._truth_returned[ids] = torch.where(
            valid, contacted & net_clear & landed_ok, self._truth_returned[ids]
        )
        self._truth_landing_xy[ids] = torch.where(
            valid.unsqueeze(-1), xy, self._truth_landing_xy[ids]
        )

    def cross_engine_physical_truth(
        self, env_id: int, *, expected_attempt_token: int, final: bool
    ) -> dict:
        """Return one explicit physical-outcome record for the BankExam scorecard.

        ``final=True`` means the one-question evaluator is about to finalize the attempt. A
        contacted ball without a recorded landing remains unavailable and must fail the cell;
        a completed swing with no contact is a valid all-false physical outcome.
        """

        if self.cross_engine_truth_capability != (
            "physical_paddle_contact_and_post_contact_flight_v1"
        ):
            return {
                "available": False,
                "capability": self.cross_engine_truth_capability,
                "reason": self.cross_engine_truth_metadata["reason"],
            }
        index = int(env_id)
        expected = int(expected_attempt_token)
        token = int(self._truth_attempt_token[index])
        if not bool(self._truth_exam_active[index]) or token != expected:
            return {
                "available": False,
                "capability": self.cross_engine_truth_capability,
                "reason": (
                    f"physical truth generation mismatch: expected token {expected}, got {token}"
                ),
            }
        if bool(self._truth_published[index]):
            available = bool(self._truth_available[index])
            contacted = bool(self._truth_contacted[index])
            net_clear = bool(self._truth_net_clear[index])
            landed_ok = bool(self._truth_landed_ok[index])
            returned = bool(self._truth_returned[index])
            landing_xy = self._truth_landing_xy[index]
            served = bool(self._truth_published_served[index])
            exact_seen = bool(self._truth_published_exact_seen[index])
        else:
            contacted = bool(self._impulse_done[index])
            landed = contacted and bool(self._landed[index])
            net_clear = landed and bool(self._net_crossed[index]) and bool(
                self._net_z[index] > self._net_clear_z
            )
            landing_xy = self._ret_land_xy[index]
            landed_ok = landed and bool(landing_xy[0] > self._net_x_env) and bool(
                landing_xy[0] <= self._near_x + self._table_len
            ) and bool(landing_xy[1].abs() <= self._half_w)
            returned = contacted and net_clear and landed_ok
            served = bool(self._truth_served[index])
            exact_seen = bool(self._truth_exact_seen[index])
            available = bool(final) and served and exact_seen and ((not contacted) or landed)
        if not available:
            if not served:
                reason = "physical incoming ball was never served for this exam attempt"
            elif not exact_seen:
                reason = "exam attempt finalized before its exact-strike frame"
            elif contacted:
                reason = "racket contact occurred but no post-contact landing was recorded"
            else:
                reason = "physical outcome pending"
            return {
                "available": False,
                "capability": self.cross_engine_truth_capability,
                "reason": reason,
                "attempt_token": token,
                "served": served,
                "exact_seen": exact_seen,
            }
        return {
            "available": True,
            "capability": self.cross_engine_truth_capability,
            "contacted": contacted,
            "net_clear": net_clear,
            "landed_ok": landed_ok,
            "returned": returned,
            "attempt_token": token,
            "served": served,
            "exact_seen": exact_seen,
            "landing_xy_env_m": [float(landing_xy[0]), float(landing_xy[1])] if contacted else None,
            "contact_authority": "code_driven_blade_disc_and_venue_paddle_impulse",
            "post_contact_rollout": "physx_gravity_plus_deterministic_venue_aero_and_code_table_bounce",
        }

    def update(self, exact_strike: torch.Tensor) -> None:
        """Once per control step from ``_update_metrics`` (after ``_vb_evaluate``)."""
        cmd = self._cmd
        origins = self._env.scene.env_origins
        step_dt = float(self._env.step_dt)

        # 1) fold bounce/landing events flagged by the physics callback since last control step.
        self._consume_events()
        if self._impulse_on:
            self._truth_exact_seen |= self._truth_exam_active & exact_strike

        # 2) serve: parked envs whose tts entered (step_dt, SERVE_HORIZON_S] are CANDIDATES.
        #    FIRST candidate step per swing runs the reverse integration (env-local frame,
        #    vb-convention) once to DISCOVER the final-ballistic-segment length tts_effective,
        #    which is cached — subsequent WAITING steps only compare tts against the cache, no
        #    re-integration. A row launches only when its remaining tts fits inside the
        #    un-truncated segment (tts <= tts_effective): un-truncated discoveries serve on the
        #    spot from that same integration; TRUNCATED rows serve LATER (one more integration on
        #    the serve step, over exactly t_back = tts <= tts_effective — un-truncated by
        #    construction), from ON the incoming trajectory, so forward flight for exactly tts
        #    seconds still arrives at the question (contact, velocity) at the exact-strike frame
        #    (tts is an exact multiple of step_dt — bank runs forbid retiming). Rows whose whole
        #    final segment is shorter than one control step never serve and are counted at the
        #    strike as pb_missed_serve. Cost: <= 2 integrations per swing (was: one per waiting
        #    step). Truncation is LATCHED here but counted only at consumption (serve/strike).
        just_served = torch.zeros_like(self._landed)
        cand = schedule_serves(self._mode == _MODE_PARKED, cmd.time_to_strike,
                               SERVE_HORIZON_S, min_tts_s=step_dt)
        discover = cand & ~self._teff_valid
        serve_cached = cand & self._teff_valid & (cmd.time_to_strike <= self._teff_cache + 1e-6)
        integ = discover | serve_cached
        if bool(integ.any()):
            t_back = cmd.time_to_strike.clamp(min=0.0, max=SERVE_HORIZON_S)
            pos_env, vel_w, t_eff = back_integrate_incoming(
                cmd.racket_target_pos_w - origins, cmd.vb_vel_in_w, cmd.vb_spin_in_w,
                t_back, self._prm, h=SERVE_BACKINT_H,
                surface_z=float(cmd.cfg.vb_table_surface_z), margin=SERVE_PLANE_MARGIN,
            )
            self._teff_cache = torch.where(discover, t_eff, self._teff_cache)
            self._teff_valid |= discover
            # 1e-4 truncation tolerance: t_eff is a float32 per-step sum (~1e-5 noise); a row
            # truly truncated within the last 1e-4 s costs <= |v|*1e-4 ~ 0.4 mm at the strike —
            # far below the 17 mm engine floor.
            due = integ & (t_eff >= t_back - 1e-4)
            self._trunc_flag |= discover & ~due  # latch only; counted at serve/strike
            just_served = due
            if bool(due.any()):
                ids = torch.where(due)[0]
                pose = torch.cat([origins[ids] + pos_env[ids], self._identity_quat[ids]], dim=-1)
                vel6 = torch.cat([vel_w[ids], cmd.vb_spin_in_w[ids]], dim=-1)
                self._ball.write_root_pose_to_sim(pose, env_ids=ids)
                self._ball.write_root_velocity_to_sim(vel6, env_ids=ids)
                self._mode[ids] = _MODE_INBOUND
                if self._impulse_on:
                    self._truth_served[ids] |= self._truth_exam_active[ids]
                self._landed[ids] = False
                self._prev_valid[ids] = False
                self._prev_dn_valid[ids] = False  # teleport -> stale blade-plane distance
                self._serve_count += float(len(ids))
                # One-per-swing truncation accounting, CONSUMED at the serve: invariant under
                # upstream resample repeats (each repeat clears the latch, the next candidate
                # step re-latches it, the single serve consumes it once).
                delayed = due & self._trunc_flag
                if bool(delayed.any()):
                    self._trunc_count += float(delayed.sum())
                self._trunc_flag = self._trunc_flag & ~due

        # 3) strike-frame truth measurement (the instrument's headline numbers). just_served envs
        #    are excluded (their write hasn't been integrated yet); with tts > step_dt at serve
        #    time this overlap cannot occur anyway. A swing whose blade already struck the ball
        #    (mode RETURN, Phase B early hit) is excluded by construction — the inbound arrival
        #    is no longer measurable (documented; the hit itself is in pb_hit_count).
        meas = exact_strike & (self._mode == _MODE_INBOUND) & ~just_served
        # Phase B: latch THIS env's analytic vb landing prediction at ITS strike frame for the
        # pb_virt_phys_gap_m cross-ruler (update() runs AFTER _vb_evaluate, so vb_fired /
        # vb_landing_xy are this step's values for exactly the exact_strike rows; the batch-wide
        # buffers are clobbered on every later strike-carrying step — shadow-driver pattern).
        if self._impulse_on and bool(exact_strike.any()):
            lat = exact_strike & self._cmd.vb_fired
            if bool(lat.any()):
                self._pred_xy = torch.where(lat.unsqueeze(-1), self._cmd.vb_landing_xy, self._pred_xy)
                self._pred_valid = self._pred_valid | (lat & self._cmd.vb_landing_valid)
        if bool(meas.any()):
            serve_err = torch.linalg.norm(
                self._ball.data.root_pos_w - cmd.racket_target_pos_w, dim=-1
            )
            vel_err = torch.linalg.norm(
                self._ball.data.root_lin_vel_w - cmd.vb_vel_in_w, dim=-1
            )
            decay = float(cmd.cfg.exact_success_decay)
            self._serve_err_acc = decay * self._serve_err_acc + float(serve_err[meas].sum())
            self._serve_vel_err_acc = decay * self._serve_vel_err_acc + float(vel_err[meas].sum())
            self._serve_n_acc = decay * self._serve_n_acc + float(meas.sum())
            self._meas_count += float(meas.sum())
            # Phase A: no racket impulse — the ball continues THROUGH the strike point/robot.
            self._mode[meas] = _MODE_POST
        # Strike frame reached while still parked (resampled inside the last control step, or the
        # question was never realizable this swing): count the unserved strike, and consume the
        # truncation latch of swings that were delayed but never got a serve window (t_eff <
        # one control step) — the OTHER once-per-swing consumption point.
        missed = exact_strike & (self._mode == _MODE_PARKED)
        if bool(missed.any()):
            self._missed_serve_count += float(missed.sum())
            late = missed & self._trunc_flag
            if bool(late.any()):
                self._trunc_count += float(late.sum())
            self._trunc_flag = self._trunc_flag & ~missed
        # Clock jumped past the strike WITHOUT an exact-strike frame (deploy-parity mid-swing clip
        # switch): the inbound flight is no longer measurable — let it fly out as POST (silently:
        # the gate was never evaluated; mirrors the shadow driver's stale handling).
        stale = (self._mode == _MODE_INBOUND) & (cmd.time_to_strike < -0.5 * step_dt) & ~meas
        if bool(stale.any()):
            self._mode[stale] = _MODE_POST

        # 4) retire post-strike/return balls that recorded their landing and fell away, then
        #    park-drive.
        done = ((self._mode == _MODE_POST) | (self._mode == _MODE_RETURN)) & (
            (self._ball.data.root_pos_w[:, 2] - origins[:, 2]) < KILL_Z_ENV
        )
        if bool(done.any()):
            self._mode[done] = _MODE_PARKED
        parked = self._mode == _MODE_PARKED
        if bool(parked.any()):
            ids = torch.where(parked)[0]
            pose = torch.cat([origins[ids] + self._park_pos_env[ids], self._identity_quat[ids]], dim=-1)
            vel6 = torch.zeros(len(ids), 6, device=self.device)
            self._ball.write_root_pose_to_sim(pose, env_ids=ids)
            self._ball.write_root_velocity_to_sim(vel6, env_ids=ids)
            self._prev_valid[parked] = False
            self._prev_dn_valid[parked] = False

        # 5) metrics (broadcast counters; land_x/y held per env at its most recent landing).
        m = cmd.metrics
        m["pb_serve_count"][:] = self._serve_count
        m["pb_strike_meas_count"][:] = self._meas_count
        m["pb_missed_serve_count"][:] = self._missed_serve_count
        m["pb_serve_truncated_count"][:] = self._trunc_count
        m["pb_bounce_count"][:] = self._bounce_count
        m["pb_land_count"][:] = self._land_count
        m["pb_land_on_table_count"][:] = self._land_on_table_count
        if self._serve_n_acc >= 1.0:
            m["pb_serve_err_m"][:] = self._serve_err_acc / self._serve_n_acc
            m["pb_serve_vel_err"][:] = self._serve_vel_err_acc / self._serve_n_acc
        if self._impulse_on:
            m["pb_hit_count"][:] = self._hit_count
            m["pb_return_count"][:] = self._ret_land_count
            m["pb_return_bounce_count"][:] = self._ret_bounce_count
            m["pb_return_net_clear_count"][:] = self._net_clear_count
            m["pb_return_net_clear_rate"][:] = self._net_clear_count / max(self._ret_land_count, 1.0)
            if self._ret_err_n_acc >= 1.0:
                m["pb_return_land_err_m"][:] = self._ret_err_acc / self._ret_err_n_acc
            if self._gap_n_acc >= 1.0:
                m["pb_virt_phys_gap_m"][:] = self._gap_acc / self._gap_n_acc
            for cohort_id, name in enumerate(self._cohort_names):
                questions = self._cohort_stats["question"][cohort_id]
                hits = self._cohort_stats["hit"][cohort_id]
                returns = self._cohort_stats["return"][cohort_id]
                land_n = self._cohort_stats["land_err_n"][cohort_id]
                gap_n = self._cohort_stats["gap_n"][cohort_id]
                m[f"pb_{name}_question_count"][:] = questions
                m[f"pb_{name}_hit_count"][:] = hits
                m[f"pb_{name}_return_count"][:] = returns
                m[f"pb_{name}_bounce_count"][:] = self._cohort_stats[
                    "bounce"
                ][cohort_id]
                m[f"pb_{name}_net_clear_rate"][:] = self._cohort_stats[
                    "net_clear"
                ][cohort_id] / returns.clamp_min(1.0)
                m[f"pb_{name}_opponent_land_rate"][:] = self._cohort_stats[
                    "opponent_land"
                ][cohort_id] / returns.clamp_min(1.0)
                m[f"pb_{name}_landing_error_m"][:] = self._cohort_stats[
                    "land_err_sum"
                ][cohort_id] / land_n.clamp_min(1.0)
                m[f"pb_{name}_analytic_physical_gap_m"][:] = self._cohort_stats[
                    "gap_sum"
                ][cohort_id] / gap_n.clamp_min(1.0)

        # 6) refresh the host-side activity counters for the substep hot path. Arming/activation
        #    only ever happens HERE (the serve above), so between control steps these can only
        #    over-estimate (callback hits/landings/kills merely disarm) — a counter-driven skip
        #    in the callback is therefore always correct, with ZERO per-substep device syncs.
        #    (on_resample only parks, which also only over-estimates until this refresh.)
        #    Cost: <= 2 host syncs per CONTROL step, replacing 2 bool(any) syncs per SUBSTEP.
        self._active_host = int((self._mode != _MODE_PARKED).sum())
        if self._impulse_on:
            armed_now = (~self._impulse_done) & (
                (self._mode == _MODE_INBOUND) | ((self._mode == _MODE_POST) & ~self._landed)
            )
            self._armed_host = int(armed_now.sum())

    # ------------------------------------------------------------------ #
    # internals
    # ------------------------------------------------------------------ #
    def _consume_events(self) -> None:
        """Fold events flagged since the last control step into the held metrics/counters."""
        new = self._land_new
        if bool(new.any()):
            m = self._cmd.metrics
            m["pb_land_x"] = torch.where(new, self._land_xy[:, 0], m["pb_land_x"])
            m["pb_land_y"] = torch.where(new, self._land_xy[:, 1], m["pb_land_y"])
            on_table = new & table_bounds_mask(
                self._land_xy, self._near_x, self._table_len, self._half_w
            )
            self._land_count += float(new.sum())
            self._land_on_table_count += float(on_table.sum())
            self._land_new.zero_()
        if bool(self._bounce_new.any()):
            self._bounce_count += float(self._bounce_new.sum())
            self._bounce_new.zero_()
        if not self._impulse_on:
            return
        # Phase B: racket hits flagged by the physics callback since the last control step.
        if bool(self._hit_new.any()):
            self._hit_count += float(self._hit_new.sum())
            self._cohort_stats["hit"] += torch.bincount(
                self._question_cohort[self._hit_new], minlength=3
            ).float()
            self._hit_new.zero_()
        # Phase B: return-flight table bounces (mode-split from pb_bounce_count — see
        # _detect_bounce_and_landing).
        if bool(self._ret_bounce_new.any()):
            self._ret_bounce_count += float(self._ret_bounce_new.sum())
            self._cohort_stats["bounce"] += torch.bincount(
                self._question_cohort[self._ret_bounce_new], minlength=3
            ).float()
            self._ret_bounce_new.zero_()
        # Phase B: return landings — landing point (held per env), error vs the question landing
        # target, net clearance (evaluated at landing so the denominator is landed returns; the
        # crossing z was latched when the return crossed the net plane), and the virt-vs-phys
        # cross-ruler where the analytic prediction was latched for the same strike.
        rnew = self._ret_land_new
        if bool(rnew.any()):
            m = self._cmd.metrics
            m["pb_return_land_x"] = torch.where(rnew, self._ret_land_xy[:, 0], m["pb_return_land_x"])
            m["pb_return_land_y"] = torch.where(rnew, self._ret_land_xy[:, 1], m["pb_return_land_y"])
            self._ret_land_count += float(rnew.sum())
            clear = rnew & self._net_crossed & (self._net_z > self._net_clear_z)
            self._net_clear_count += float(clear.sum())
            on_opponent = (
                rnew
                & table_bounds_mask(
                    self._ret_land_xy,
                    self._near_x,
                    self._table_len,
                    self._half_w,
                )
                & (self._ret_land_xy[:, 0] > self._net_x_env)
            )
            self._cohort_stats["return"] += torch.bincount(
                self._question_cohort[rnew], minlength=3
            ).float()
            self._cohort_stats["net_clear"] += torch.bincount(
                self._question_cohort[clear], minlength=3
            ).float()
            self._cohort_stats["opponent_land"] += torch.bincount(
                self._question_cohort[on_opponent], minlength=3
            ).float()
            decay = float(self._cmd.cfg.exact_success_decay)
            err = torch.linalg.norm(
                self._ret_land_xy - self._ret_target_xy, dim=-1
            )
            self._ret_err_acc = decay * self._ret_err_acc + float(err[rnew].sum())
            self._ret_err_n_acc = decay * self._ret_err_n_acc + float(rnew.sum())
            cohort = self._question_cohort[rnew]
            self._cohort_stats["land_err_sum"].scatter_add_(
                0, cohort, err[rnew]
            )
            self._cohort_stats["land_err_n"] += torch.bincount(
                cohort, minlength=3
            ).float()
            both = rnew & self._pred_valid
            if bool(both.any()):
                gap = torch.linalg.norm(self._ret_land_xy - self._pred_xy, dim=-1)
                self._gap_acc = decay * self._gap_acc + float(gap[both].sum())
                self._gap_n_acc = decay * self._gap_n_acc + float(both.sum())
                cohort = self._question_cohort[both]
                self._cohort_stats["gap_sum"].scatter_add_(
                    0, cohort, gap[both]
                )
                self._cohort_stats["gap_n"] += torch.bincount(
                    cohort, minlength=3
                ).float()
            self._ret_land_new.zero_()

    def _detect_bounce_and_landing(self) -> None:
        """Descending surface+R crossing scan on the current vs previous ball sample (any rate).

        In-bounds crossings get the CODE-DRIVEN venue table bounce (velocity/spin rewritten, ball
        snapped back to the plane at the interpolated crossing point). The first POST-strike
        crossing (in-bounds or not — the shadow/vb landing-plane convention) is recorded as the
        landing. Pre-strike in-bounds crossings bounce too (physical consistency; see module
        docstring — cannot occur for in-envelope questions).

        BOUNCE COUNTING IS MODE-SPLIT (adversarial-review fix): every in-bounds crossing still
        takes the physical bounce, but only non-RETURN crossings latch ``_bounce_new`` (the Phase
        A pre-strike honesty counter, ~0 by construction) while RETURN-flight crossings latch
        ``_ret_bounce_new`` -> ``pb_return_bounce_count`` (returns legitimately bounce on the
        opponent half — without the split they dominated pb_bounce_count and inverted its
        invariant).

        HOT-PATH SYNC DISCIPLINE: the idle early-out uses the host-side ``_active_host`` counter
        (control-rate maintained, over-estimate-only — see update()) and all event LATCHES are
        branchless masked writes; the only per-substep ``bool()`` host sync left is the rare
        bounce sim-write branch, which needs env_ids extraction.
        """
        if self._active_host == 0:
            self._prev_valid.zero_()
            return
        active = self._mode != _MODE_PARKED
        origins = self._env.scene.env_origins
        pos_env = self._ball.data.root_pos_w - origins

        # Phase B: net-plane crossing scan for RETURN flights (before the landing record below so
        # a segment that crosses the net AND the table plane in one step counts both). Latched
        # once per swing; the clearance verdict is taken at landing consumption. Branchless.
        if self._impulse_on:
            ncross, z_at = net_plane_crossing(self._prev_pos_env, pos_env, self._net_x_env)
            nc = (
                (self._mode == _MODE_RETURN)
                & self._prev_valid
                & ~self._net_crossed
                & ~self._landed
                & ncross
            )
            self._net_z = torch.where(nc, z_at, self._net_z)
            self._net_crossed |= nc

        crossed, xy = _sb.landing_crossing(self._prev_pos_env, pos_env, self._z_thr)
        evt = active & self._prev_valid & crossed

        # landing record: first post-strike crossing (branchless masked latch).
        land = evt & (self._mode == _MODE_POST) & ~self._landed
        self._land_xy = torch.where(land.unsqueeze(-1), xy, self._land_xy)
        # Phase B: first crossing of a RETURN flight = the return landing (same plane
        # convention, in-bounds or not; folded into the pb_return_* metrics at control rate).
        ret = evt & (self._mode == _MODE_RETURN) & ~self._landed
        self._ret_land_xy = torch.where(ret.unsqueeze(-1), xy, self._ret_land_xy)
        self._landed |= land | ret
        self._land_new |= land
        self._ret_land_new |= ret

        # code-driven bounce: in-bounds crossings only (off the ends/sides the ball just
        # keeps falling toward the floor — no floor model, it parks at KILL_Z_ENV).
        bounce = evt & table_bounds_mask(xy, self._near_x, self._table_len, self._half_w)
        # Mode-split counting (see docstring). With impulse off, mode never reaches RETURN, so
        # the non-RETURN gate is the identity — Phase A byte-parity holds.
        self._bounce_new |= bounce & (self._mode != _MODE_RETURN)
        if self._impulse_on:
            self._ret_bounce_new |= bounce & (self._mode == _MODE_RETURN)
        if bool(bounce.any()):  # sim write needs env_ids -> the ONE remaining substep sync
            v_minus = self._ball.data.root_lin_vel_w
            w_minus = self._ball.data.root_ang_vel_w
            v_plus, w_plus = predict_table_contact(v_minus, w_minus, self._tp)
            ids = torch.where(bounce)[0]
            new_pos_env = pos_env.clone()
            new_pos_env[ids, 0] = xy[ids, 0]
            new_pos_env[ids, 1] = xy[ids, 1]
            new_pos_env[ids, 2] = self._z_thr
            pose = torch.cat(
                [origins[ids] + new_pos_env[ids], self._ball.data.root_quat_w[ids]], dim=-1
            )
            vel6 = torch.cat([v_plus[ids], w_plus[ids]], dim=-1)
            self._ball.write_root_pose_to_sim(pose, env_ids=ids)
            self._ball.write_root_velocity_to_sim(vel6, env_ids=ids)
            # compare-from state for the next scan = the snapped-back position.
            pos_env = torch.where(bounce.unsqueeze(-1), new_pos_env, pos_env)

        self._prev_pos_env.copy_(pos_env)
        self._prev_valid.copy_(active)

    def _detect_racket_impulse(self) -> None:
        """Phase B: blade-disc contact scan + the CODE-DRIVEN racket impulse (any rate).

        DETECTION RATE — physics substep (200 Hz), like the table bounce and the reference
        table_tennis_env._handle_paddle: at up to ~10 m/s ball-blade closing speed a 50 Hz
        control step moves ~20 cm (>> the 7.5 cm blade), so control-rate detection would miss
        most hits; at 5 ms substeps the motion is ~5 cm and the sign-crossing branch of
        :func:`blade_disc_contact` covers the remainder (no CCD needed — see the init print).

        BLADE STATE — the command's PURE FK helper (``RacketTargetCommand._racket_fk()``:
        articulation data is lazily refreshed against the sim timestamp, so the blade pose read
        here is substep-fresh) returns ``(pos, quat, lin_vel, raw_normal, signed_normal)`` as
        LOCALS — the same math as the reward path, NO fourth FK derivation, and NO write to the
        command's ``racket_pos_w`` / ``racket_quat_w`` / ``racket_lin_vel_w`` /
        ``racket_normal_raw_w`` / ``racket_normal_w`` buffers. This is load-bearing
        (adversarial-review fix): Isaac Lab 2.1 runs
        ``reward_manager.compute`` BEFORE ``command_manager.compute``, so a substep rebind of
        those buffers would feed the racket tracking rewards mid-step-fresh FK (~one control
        frame ahead of the flag-off baseline) — a metrics-only contract violation. Regression:
        all five attributes must stay the IDENTICAL tensor objects across this scan
        (test_physical_ball_helpers.test_substep_fk_scan_side_effect_free).

        IMPULSE — :func:`racket_impulse` (pure delegation to virtual_ball.predict_paddle_contact,
        the venue e(u_n) paddle model) on the ball's ENGINE state at the contact substep; the
        outgoing (v+, omega+) is written via ``write_root_velocity_to_sim`` (WORLD frame — the
        one frame trap: external wrenches are body-frame in Isaac Lab 2.1, root-velocity writes
        are world-frame) TOGETHER WITH the blade-plane contact-point snap via
        ``write_root_pose_to_sim`` (the table-bounce snap mirrored; adversarial-review fix — the
        detection sample can sit 2-5 cm past the plane on the crossing branch, and launching the
        return from there biased pb_return_land_* / pb_virt_phys_gap_m by that offset): crossing
        rows interpolate the substep fraction ``f = prev_d_n / (prev_d_n - d_n)`` along the
        prev->current ball segment to the ``d_n = 0`` plane point (the vb channel's blade-plane
        convention — its rollout starts at racket_pos_w ON the plane); slab rows (no sign
        change) project along the face normal onto the plane. One impulse per swing
        (``_impulse_done``), re-armed by on_resample.

        HOT-PATH SYNC DISCIPLINE: the idle early-out uses the host-side ``_armed_host`` counter
        (control-rate maintained, over-estimate-only — see update()), skipping the FK + scan
        with no device sync; the only per-substep ``bool()`` host sync left is the rare hit
        sim-write branch, which needs env_ids extraction.
        """
        if self._armed_host == 0:
            self._prev_dn_valid.zero_()
            return
        armed = (~self._impulse_done) & (
            (self._mode == _MODE_INBOUND) | ((self._mode == _MODE_POST) & ~self._landed)
        )
        cmd = self._cmd
        # PURE FK locals (never rebind cmd.racket_* — see docstring).
        r_pos, _r_quat, r_lin_vel, _r_normal_raw, r_normal = cmd._racket_fk()
        ball_pos = self._ball.data.root_pos_w
        ball_vel = self._ball.data.root_lin_vel_w
        prev_dn = self._prev_dn
        prev_ok = self._prev_dn_valid & armed
        hit, d_n = blade_disc_contact(
            ball_pos,
            ball_vel,
            r_pos,
            r_lin_vel,
            r_normal,
            prev_dn,
            prev_ok,
            racket_radius=RACKET_CONTACT_RADIUS,
            ball_radius=float(self._prm.ball_radius),
        )
        hit = hit & armed
        if bool(hit.any()):  # sim write needs env_ids -> the ONE remaining substep sync
            v_plus, w_plus = racket_impulse(
                ball_vel,
                r_lin_vel,
                r_normal,
                self._ball.data.root_ang_vel_w,
                self._prm,
            )
            # Contact-point snap (see docstring): reconstruct the blade-PLANE (d_n = 0) contact
            # position. Crossing rows: interpolate the substep fraction along the prev->current
            # ball segment (prev position from the bounce scan's compare-from state, same
            # sampling cadence as prev_dn). Slab rows / no valid prev: project along the normal.
            n_hat = r_normal / (torch.linalg.norm(r_normal, dim=-1, keepdim=True) + 1e-9)
            origins = self._env.scene.env_origins
            prev_pos_w = self._prev_pos_env + origins
            denom = prev_dn - d_n
            safe_denom = torch.where(denom.abs() > 1e-9, denom, torch.full_like(denom, 1e-9))
            f = (prev_dn / safe_denom).clamp(0.0, 1.0).unsqueeze(-1)
            interp = prev_pos_w + (ball_pos - prev_pos_w) * f
            proj = ball_pos - d_n.unsqueeze(-1) * n_hat
            use_interp = prev_ok & self._prev_valid & ((prev_dn * d_n) < 0.0)
            contact_w = torch.where(use_interp.unsqueeze(-1), interp, proj)
            ids = torch.where(hit)[0]
            pose = torch.cat([contact_w[ids], self._ball.data.root_quat_w[ids]], dim=-1)
            vel6 = torch.cat([v_plus[ids], w_plus[ids]], dim=-1)
            self._ball.write_root_pose_to_sim(pose, env_ids=ids)
            self._ball.write_root_velocity_to_sim(vel6, env_ids=ids)  # WORLD frame
            # compare-from state for the same-substep bounce/net scan = the snapped contact
            # point (the table-bounce snap's :func:`_detect_bounce_and_landing` convention).
            self._prev_pos_env[ids] = contact_w[ids] - origins[ids]
            self._mode[ids] = _MODE_RETURN
            self._impulse_done |= hit
            self._hit_new |= hit
        self._prev_dn = d_n
        self._prev_dn_valid = armed & ~hit

    def _on_physics_step(self, dt: float) -> None:
        """Physics-substep callback (table_tennis_env.py mechanism): aero wrench + impulse scan
        + bounce scan.

        Asset ``data`` buffers are lazily refreshed against the sim timestamp, so reads here are
        per-substep fresh. The wrench is written for the FULL batch every substep (zeros where
        parked) so a just-parked ball never keeps a stale external force. Order: aero wrench
        (pre-impulse velocity; the next substep sees the corrected state — the same one-substep
        lag the table bounce always had), then the Phase B impulse scan, then the bounce/landing
        scan (position-based, unaffected by the same-substep velocity rewrite).
        """
        active = self._mode != _MODE_PARKED
        lin_vel_w = self._ball.data.root_lin_vel_w
        ang_vel_w = self._ball.data.root_ang_vel_w
        if self._substep > 1:
            force_w = substepped_aero_force(
                lin_vel_w, ang_vel_w, self._mass, self._prm, dt, self._substep
            )
        else:
            force_w = _sb.venue_aero_force(
                lin_vel_w, ang_vel_w, self._mass, self._prm.k_d, self._prm.k_m
            )
        force_w = force_w * active.unsqueeze(-1)
        # Isaac Lab 2.1 applies this BODY-frame wrench at the link transform origin.  The
        # origin-centred SphereCfg has zero COM offset, so the point is also the ball COM.
        self._force_b[:, 0, :] = _sb.quat_rotate_inverse_wxyz(self._ball.data.root_quat_w, force_w)
        self._ball.set_external_force_and_torque(self._force_b, self._torque_b)
        self._ball.write_data_to_sim()

        if self._impulse_on:
            self._detect_racket_impulse()
        self._detect_bounce_and_landing()
