"""SHADOW physical ball (+ optional table) for the tracking task — MEASUREMENT ONLY.

WHAT THIS IS: a flag-gated (``RacketTargetCommandCfg.shadow_ball``) real PhysX rigid-body ball,
one per env, that (a) flies in kinematically along the current question's incoming straight-line
path so it reaches the question contact point exactly at the strike frame, (b) at a CAPTURED
strike (the same ``exact_strike`` + capture-gate signal the reward path uses, i.e.
``RacketTargetCommand.vb_fired``) is handed to PhysX with the post-contact velocity/spin from the
SAME venue paddle-contact model the reward path evaluates
(:func:`virtual_ball.predict_paddle_contact` on the ACHIEVED racket FK state), and (c) then flies
dynamically under PhysX gravity + the per-physics-substep venue aero wrench until its landing
(first descending crossing of ``vb_table_surface_z + ball_radius``, env-local) is recorded.

WHAT THIS IS NOT: it is a SHADOW / measurement channel only. It never touches rewards,
observations, or the question-bank target override logic — the analytic virtual ball
(:mod:`virtual_ball`) stays the reward machine. The value of the shadow ball is that every
training strike becomes an online engine-vs-analytic cross-check: ``shadow_vs_virtual_land_err``
compares the PhysX-integrated landing against the coarse-RK4 virtual-ball landing prediction
snapshotted at the same strike, with identical initial conditions (same start point = achieved
racket center, same contact model, same aero law).

HONESTY NOTES (read before trusting the numbers):

* The PRE-STRIKE segment is COSMETIC. Questions are defined AT the contact point (position +
  incoming velocity there), so the approach is a LINEAR backward extrapolation
  ``p(tts) = contact - v_in * tts`` (no drag/Magnus on the way in), clamped to
  ``PRESTRIKE_HORIZON_S`` so far-from-strike envs don't park the ball tens of meters away. The
  physics test is the POST-strike flight only.
* Bounce-before-strike is NOT modeled: the incoming ball materializes on the straight-line path;
  there is no serve, no incoming table bounce (questions are defined at contact).
* The standalone engine-fidelity number (PhysX + this aero injection vs the venue RK4 reference,
  no robot in the loop) comes from ``scripts/isaac_ball_inloop_check.py`` — use that to separate
  "engine integrates the venue model correctly" from "the analytic landing prediction disagrees
  with the engine for achieved strikes".
* With ``shadow_table=True`` the ball collider is enabled so it can physically bounce on the
  static table collider; the dynamic handoff then spawns the ball ``ball_radius +
  SPAWN_CLEARANCE`` along the oriented contact normal off the racket center so it never starts
  inside the racket collision mesh (a ~3 cm start offset vs the virtual rollout, which starts at
  the racket center exactly). With ``shadow_table=False`` (collider disabled) the start point is
  byte-matched to the virtual rollout.
* Aero wrench mechanism: identical to ``table_tennis/table_tennis_env.py`` — a
  ``sim.add_physics_callback`` registered per-substep callback that reads the ball state, computes
  the venue aero force, rotates it WORLD -> BODY (Isaac Lab 2.1 ``set_external_force_and_torque``
  applies wrenches in the body frame at the link transform origin; this origin-centred
  ``SphereCfg`` has zero COM offset, so origin and COM coincide; ``is_global`` only exists in
  >= 2.2 — see table_tennis_env.py and isaac_ball_inloop_check.py), and writes it to the sim.

This module is importable WITHOUT Isaac (top-level imports are torch-only); the driver lazy-loads
``virtual_ball`` through the package at runtime. The pure helpers below are unit-tested Isaac-free
in ``tests/test_shadow_ball_helpers.py``.
"""

from __future__ import annotations

import torch

# Pre-strike kinematic path: only the last PRESTRIKE_HORIZON_S seconds of the incoming flight are
# rendered (linear backward extrapolation from the contact point). Cosmetic bound: keeps the ball
# within its own env footprint (cross-env collisions are filtered by Isaac Lab env cloning anyway).
PRESTRIKE_HORIZON_S = 0.6
# Dynamic-handoff clearance off the racket face along the oriented contact normal, applied ONLY
# when the ball collider is enabled (shadow_table=True) so the ball never spawns inside the racket
# collision mesh. Total offset = ball_radius + SPAWN_CLEARANCE.
SPAWN_CLEARANCE = 0.01
# Park position (env-local) for missed strikes / idle balls: well below the floor, out of sight.
# Rewritten kinematically every control step, so terrain depenetration can never accumulate.
PARK_POS_ENV = (0.0, 0.0, -10.0)
# Bounce detection band (m) around the contact plane for the table-collider case, where PhysX
# resolves the contact and the ball center may never sample below surface_z + R (see
# bounce_detect). Only used when shadow_table=True.
BOUNCE_BAND = 0.03

_MODE_PRE = 0     # kinematic incoming flight (root state rewritten every control step)
_MODE_POST = 1    # dynamic PhysX flight after a captured strike (aero wrench per substep)
_MODE_PARKED = 2  # missed strike / idle: parked below the floor until the next question


# --------------------------------------------------------------------------------------------- #
# Pure helpers (torch-only; unit-tested Isaac-free)
# --------------------------------------------------------------------------------------------- #
def prestrike_ball_state(
    contact_pos_w: torch.Tensor,
    v_in_w: torch.Tensor,
    tts: torch.Tensor,
    horizon_s: float = PRESTRIKE_HORIZON_S,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Kinematic incoming-ball state at ``time_to_strike`` seconds before contact.

    LINEAR backward extrapolation (documented choice): ``p = contact - v_in * clamp(tts, 0, H)``.
    The venue flight model is deliberately NOT integrated backwards here — the pre-strike segment
    is cosmetic (questions are defined at the contact point; the physics test is post-strike), and
    the linear path hits the contact point exactly at tts=0 by construction, which is the only
    property the measurement needs. ``tts`` clamped at 0 also parks late/held envs at the contact
    point instead of overshooting past it.

    Args:
        contact_pos_w: (N, 3) question contact point (world frame).
        v_in_w: (N, 3) question incoming velocity at contact (world frame).
        tts: (N,) time to strike in seconds.
        horizon_s: only the last ``horizon_s`` seconds of the approach are rendered.

    Returns:
        ``(pos_w, vel_w)``: (N, 3) ball position and velocity to write this control step.
    """
    t_back = tts.clamp(min=0.0, max=float(horizon_s)).unsqueeze(-1)
    return contact_pos_w - v_in_w * t_back, v_in_w


def landing_crossing(
    prev_pos: torch.Tensor, new_pos: torch.Tensor, z_thr: float
) -> tuple[torch.Tensor, torch.Tensor]:
    """First DESCENDING crossing of the horizontal plane ``z = z_thr`` between two samples.

    Same linear-interpolation crossing extraction as ``virtual_ball.coarse_landing`` (the analytic
    reference this measurement is compared against), so the two landing extractors share one
    convention. Positions may be in any translation-consistent frame (the caller uses env-local).

    Returns:
        ``(crossed, xy)``: (N,) bool mask, (N, 2) interpolated crossing point (only meaningful
        where ``crossed``).
    """
    crossed = (prev_pos[:, 2] > z_thr) & (new_pos[:, 2] <= z_thr)
    denom = (prev_pos[:, 2] - new_pos[:, 2]).clamp(min=1e-9)
    f = ((prev_pos[:, 2] - z_thr) / denom).clamp(0.0, 1.0)
    xy = prev_pos[:, :2] + (new_pos[:, :2] - prev_pos[:, :2]) * f.unsqueeze(-1)
    return crossed, xy


def bounce_detect(
    z_new: torch.Tensor,
    vz_prev: torch.Tensor,
    vz_new: torch.Tensor,
    z_thr: float,
    band: float = BOUNCE_BAND,
) -> torch.Tensor:
    """Table-collider bounce detection: vertical-velocity sign flip near the contact plane.

    With the table collider enabled PhysX resolves the contact, so the ball center may NEVER
    sample below ``z_thr = surface + R`` between two substeps (descend 5 mm above the plane,
    ascend 3 mm above it after the impulse) and the plane-crossing test misses the landing. A
    bounce is instead flagged when v_z flips negative -> non-negative within ``band`` of the
    plane. The landing point is then the current xy (sub-substep interpolation is meaningless
    across a contact impulse).
    """
    return (
        (vz_prev < 0.0)
        & (vz_new >= 0.0)
        & (z_new < z_thr + band)
        & (z_new > z_thr - band)
    )


def venue_aero_force(
    lin_vel_w: torch.Tensor,
    ang_vel_w: torch.Tensor,
    mass: float,
    k_d: float,
    k_m: float,
    speed_clip: float = 50.0,
) -> torch.Tensor:
    """Venue aero force ``F = m * (-k_d |v| v + k_m (omega x v))`` in the WORLD frame.

    This is exactly ``mass * (virtual_ball.flight_accel + g z_hat)`` — gravity is PhysX's job, the
    wrench supplies only the drag + Magnus terms of the venue flight model (k_d/k_m are
    ACCELERATION coefficients from configs/ball_physics_venue.yaml). The |v| clip mirrors
    ``table_tennis/ball.compute_aero_wrench`` (bounds the force under a numerical blowup by
    rescaling the velocity vector to the clipped magnitude). Torque is zero: no aerodynamic
    spin decay, matching the fit's omega-constant-in-flight assumption (ball angular damping 0,
    gyroscopic forces off).
    """
    speed_raw = torch.linalg.norm(lin_vel_w, dim=-1, keepdim=True)
    speed = speed_raw.clamp(max=speed_clip)
    vel_clipped = lin_vel_w * (speed / speed_raw.clamp(min=1e-8))
    return mass * (-k_d * speed * vel_clipped + k_m * torch.cross(ang_vel_w, lin_vel_w, dim=-1))


def quat_rotate_inverse_wxyz(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate ``v`` by the INVERSE of quaternion ``q`` (w, x, y, z) — world -> body frame.

    Pure-torch local implementation (identical to isaaclab.utils.math.quat_rotate_inverse) so this
    module stays importable and unit-testable without Isaac. Needed because Isaac Lab 2.1
    ``set_external_force_and_torque`` applies wrenches in the BODY frame (see
    table_tennis_env.py / isaac_ball_inloop_check.py).
    """
    w = q[:, 0:1]
    xyz = q[:, 1:4]
    t = 2.0 * torch.cross(xyz, v, dim=-1)
    return v - w * t + torch.cross(xyz, t, dim=-1)


def shadow_vs_virtual_err(shadow_xy: torch.Tensor, virtual_xy: torch.Tensor) -> torch.Tensor:
    """Planar distance (m) between the engine-integrated and analytic landing points, per env."""
    return torch.linalg.norm(shadow_xy - virtual_xy, dim=-1)


# --------------------------------------------------------------------------------------------- #
# Driver (owned/called by RacketTargetCommand when cfg.shadow_ball is on)
# --------------------------------------------------------------------------------------------- #
class ShadowBallDriver:
    """Drives the per-env shadow ball through its PRE -> POST/PARKED lifecycle. Metrics only.

    Hook points (all inside RacketTargetCommand, mirroring how ``_vb_evaluate`` hooks in):

    * ``update(exact_strike)`` — once per control step from ``_update_metrics``, AFTER
      ``_vb_evaluate`` (so ``vb_fired`` and the fresh per-env virtual landing prediction are
      available to snapshot).
    * ``on_resample(env_ids)`` — from ``_resample_command`` (episode reset AND intra-episode clip
      wrap): the env's next question starts, the ball returns to the kinematic incoming path.
    * ``_on_physics_step(dt)`` — per physics substep via ``sim.add_physics_callback`` (the
      table_tennis_env.py mechanism): venue aero wrench on post-strike balls + landing detection
      at the physics rate.
    """

    def __init__(self, command, env):
        self._cmd = command
        self._env = env
        self.device = command.device
        n = command.num_envs

        try:
            self._ball = env.scene["shadow_ball"]
        except KeyError as exc:
            raise KeyError(
                "ShadowBallDriver: scene entity 'shadow_ball' not found. cfg.shadow_ball=True "
                "requires the scene attachment from hope_env_cfg.attach_shadow_ball_scene "
                "(run automatically by HOPEPingPongAgibotA3EnvCfg.__post_init__ or the train.py "
                "racket.shadow_ball override translation)."
            ) from exc

        # Venue constants: flight (k_d/k_m) via the same loader as the reward path; mass read from
        # the same YAML (VirtualBallParams carries radius but not mass).
        from whole_body_tracking.tasks.tracking.mdp import virtual_ball as _vb

        self._vb = _vb
        self._prm = _vb.load_venue_params()
        import yaml as _yaml

        with open(_vb.default_venue_yaml_path(), "r") as fh:
            self._mass = float(_yaml.safe_load(fh)["ball"]["mass"])
        self._table_enabled = bool(command.cfg.shadow_table)
        self._z_thr = float(command.cfg.vb_table_surface_z) + float(self._prm.ball_radius)

        # Lifecycle + landing buffers.
        self._mode = torch.full((n,), _MODE_PARKED, dtype=torch.long, device=self.device)
        self._landed = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._land_new = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._prev_valid = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._prev_pos_env = torch.zeros(n, 3, device=self.device)
        self._prev_vz = torch.zeros(n, device=self.device)
        # Engine-integrated landing (env-local xy) + the analytic prediction snapshotted at the
        # SAME strike (vb_landing_xy is clobbered batch-wide on every strike-carrying step, so it
        # must be latched per env at ITS strike frame).
        self.shadow_land_xy = torch.zeros(n, 2, device=self.device)
        self.shadow_land_valid = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._pred_xy = torch.zeros(n, 2, device=self.device)
        self._pred_valid = torch.zeros(n, dtype=torch.bool, device=self.device)
        # Reusable wrench buffers (num_envs, 1 body, 3), zeroed like table_tennis_env.py.
        self._force_b = torch.zeros(n, 1, 3, device=self.device)
        self._torque_b = torch.zeros(n, 1, 3, device=self.device)
        self._identity_quat = torch.zeros(n, 4, device=self.device)
        self._identity_quat[:, 0] = 1.0
        self._park_pos_env = torch.tensor(PARK_POS_ENV, device=self.device).expand(n, 3)

        # Cumulative counters + landing-error EMA (sample-weighted, vb-metric discipline: decays
        # only on landing-carrying steps — exact at large env counts, slightly stale at small).
        self._hit_count = 0.0
        self._miss_count = 0.0
        self._err_acc = 0.0
        self._err_n_acc = 0.0
        m = command.metrics
        m["shadow_hit_count"] = torch.zeros(n, device=self.device)
        m["shadow_miss_count"] = torch.zeros(n, device=self.device)
        m["shadow_land_x"] = torch.zeros(n, device=self.device)
        m["shadow_land_y"] = torch.zeros(n, device=self.device)
        m["shadow_vs_virtual_land_err"] = torch.zeros(n, device=self.device)

        # Per-substep aero + landing detection via the table_tennis_env.py physics-callback
        # mechanism. Defensive: if it cannot register, the shadow ball still flies on PhysX
        # gravity alone (a valid, drag-free measurement scene) — never block training.
        self._aero_active = False
        try:
            env.sim.add_physics_callback("hope_shadow_ball", self._on_physics_step)
            self._aero_active = True
        except Exception as exc:  # pragma: no cover - environment-dependent
            print(
                f"[ShadowBallDriver] could not register the physics callback ({exc!r}); "
                "shadow ball flies on PhysX gravity only and landings are detected at the "
                "CONTROL rate (degraded measurement).",
                flush=True,
            )
        print(
            f"[ShadowBallDriver] SHADOW ball ON (metrics-only): R={self._prm.ball_radius} m, "
            f"mass={self._mass} kg, k_d={self._prm.k_d}, k_m={self._prm.k_m}, "
            f"table={'ON (collider enabled)' if self._table_enabled else 'OFF (collider disabled)'}, "
            f"landing plane z={self._z_thr:.4f} (env-local)",
            flush=True,
        )

    # ------------------------------------------------------------------ #
    # control-rate hooks
    # ------------------------------------------------------------------ #
    def on_resample(self, env_ids) -> None:
        """New question for these envs (reset or clip wrap): back to the kinematic incoming path.

        A post-strike flight still in the air is cut short (no landing recorded for that swing);
        the miss/landing accounting for it stays as-is.
        """
        ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        self._mode[ids] = _MODE_PRE
        self._landed[ids] = False
        self._land_new[ids] = False
        self._prev_valid[ids] = False
        self._pred_valid[ids] = False

    def update(self, exact_strike: torch.Tensor) -> None:
        """Once per control step from ``_update_metrics``, right after ``_vb_evaluate``."""
        cmd = self._cmd
        origins = self._env.scene.env_origins

        # 1) consume landings detected by the physics callback since the last control step.
        self._consume_landings(origins)

        # 2) strike transitions. vb_fired is THIS step's capture gate (exact_strike & pos_err <
        #    capture radius & approach speed) computed by _vb_evaluate on the achieved racket
        #    state; it is all-False on strike-free steps.
        fired = cmd.vb_fired & (self._mode == _MODE_PRE)
        missed = exact_strike & ~cmd.vb_fired & (self._mode == _MODE_PRE)
        if bool(fired.any()):
            self._hand_to_physx(fired)
        if bool(missed.any()):
            # Capture gate failed: the incoming ball flies through untouched. Mark the miss and
            # park it below the floor until the next question (despawn).
            self._mode[missed] = _MODE_PARKED
            self._miss_count += float(missed.sum())
        # Clock jumped past the strike WITHOUT an exact-strike frame (e.g. the deploy-parity
        # mid-swing clip switch teleports the reference clock): nothing to measure this swing —
        # park silently (NOT a capture miss; the gate was never evaluated). Normal swings never
        # trip this: their fired/missed transition happens on the exact-strike step itself.
        stale = (self._mode == _MODE_PRE) & (cmd.time_to_strike < -0.5 * float(self._env.step_dt))
        if bool(stale.any()):
            self._mode[stale] = _MODE_PARKED

        # 3) kinematic drive: PRE envs along the incoming path, PARKED envs below the floor.
        pre = self._mode == _MODE_PRE
        parked = self._mode == _MODE_PARKED
        drive = pre | parked
        if bool(drive.any()):
            pos_w, vel_w = prestrike_ball_state(
                cmd.racket_target_pos_w, cmd.vb_vel_in_w, cmd.time_to_strike
            )
            pos_w = torch.where(pre.unsqueeze(-1), pos_w, origins + self._park_pos_env)
            vel_w = torch.where(pre.unsqueeze(-1), vel_w, torch.zeros_like(vel_w))
            ids = torch.where(drive)[0]
            pose = torch.cat([pos_w[ids], self._identity_quat[ids]], dim=-1)
            vel6 = torch.cat([vel_w[ids], torch.zeros(len(ids), 3, device=self.device)], dim=-1)
            self._ball.write_root_pose_to_sim(pose, env_ids=ids)
            self._ball.write_root_velocity_to_sim(vel6, env_ids=ids)

        # 4) metrics (broadcast counters; land_x/y are held per env at its most recent landing).
        cmd.metrics["shadow_hit_count"][:] = self._hit_count
        cmd.metrics["shadow_miss_count"][:] = self._miss_count
        if self._err_n_acc >= 1.0:
            cmd.metrics["shadow_vs_virtual_land_err"][:] = self._err_acc / self._err_n_acc

        # Degraded fallback: no physics callback -> detect landings at the control rate here.
        if not self._aero_active:
            self._detect_landings(origins)

    # ------------------------------------------------------------------ #
    # internals
    # ------------------------------------------------------------------ #
    def _hand_to_physx(self, fired: torch.Tensor) -> None:
        """Captured strike: same contact model as the reward path, then PhysX owns the ball.

        Full-batch compute + masked write, mirroring ``_vb_evaluate`` (kernel-launch bound,
        batch-size independent). Inputs are byte-identical to the reward path's contact call:
        per-swing incoming (v_in, omega_in) + the ACHIEVED racket FK state at this exact-strike
        frame.
        """
        cmd = self._cmd
        v_plus, w_plus = self._vb.predict_paddle_contact(
            cmd.vb_vel_in_w, cmd.racket_lin_vel_w, cmd.racket_normal_w, cmd.vb_spin_in_w, self._prm
        )
        pos_w = cmd.racket_pos_w
        if self._table_enabled:
            # Collider enabled: spawn off the face along the oriented contact normal so the ball
            # never starts inside the racket collision mesh (documented start-offset caveat).
            n_or = self._vb.orient_normal(cmd.racket_normal_w, cmd.vb_vel_in_w, cmd.racket_lin_vel_w)
            pos_w = pos_w + n_or * (float(self._prm.ball_radius) + SPAWN_CLEARANCE)

        ids = torch.where(fired)[0]
        pose = torch.cat([pos_w[ids], self._identity_quat[ids]], dim=-1)
        vel6 = torch.cat([v_plus[ids], w_plus[ids]], dim=-1)
        self._ball.write_root_pose_to_sim(pose, env_ids=ids)
        self._ball.write_root_velocity_to_sim(vel6, env_ids=ids)

        self._mode[ids] = _MODE_POST
        self._landed[ids] = False
        self._prev_valid[ids] = False
        self._hit_count += float(len(ids))
        # Snapshot THIS env's analytic landing prediction at ITS strike (vb_landing_xy is a
        # batch-wide buffer recomputed on every strike-carrying step; only the fired rows are
        # this env's own prediction right now).
        self._pred_xy[ids] = cmd.vb_landing_xy[ids]
        self._pred_valid[ids] = cmd.vb_landing_valid[ids]

    def _detect_landings(self, origins: torch.Tensor) -> None:
        """Landing detection on the current ball state vs the previous sample (any rate)."""
        active = (self._mode == _MODE_POST) & ~self._landed
        if not bool(active.any()):
            self._prev_valid.zero_()
            return
        pos_env = self._ball.data.root_pos_w - origins
        vz = self._ball.data.root_lin_vel_w[:, 2]
        crossed, xy = landing_crossing(self._prev_pos_env, pos_env, self._z_thr)
        hit = crossed
        if self._table_enabled:
            hit = hit | bounce_detect(pos_env[:, 2], self._prev_vz, vz, self._z_thr)
        new_land = active & self._prev_valid & hit
        if bool(new_land.any()):
            # crossing rows get the interpolated point; bounce-only rows the current xy.
            land_xy = torch.where(crossed.unsqueeze(-1), xy, pos_env[:, :2])
            self.shadow_land_xy = torch.where(new_land.unsqueeze(-1), land_xy, self.shadow_land_xy)
            self._landed |= new_land
            self._land_new |= new_land
        self._prev_pos_env.copy_(pos_env)
        self._prev_vz.copy_(vz)
        self._prev_valid.copy_(active & ~self._landed)

    def _consume_landings(self, origins: torch.Tensor) -> None:
        """Fold landings flagged since the last control step into the buffers/metrics."""
        new = self._land_new
        if not bool(new.any()):
            return
        cmd = self._cmd
        self.shadow_land_valid |= new
        cmd.metrics["shadow_land_x"] = torch.where(
            new, self.shadow_land_xy[:, 0], cmd.metrics["shadow_land_x"]
        )
        cmd.metrics["shadow_land_y"] = torch.where(
            new, self.shadow_land_xy[:, 1], cmd.metrics["shadow_land_y"]
        )
        both = new & self._pred_valid
        if bool(both.any()):
            err = shadow_vs_virtual_err(self.shadow_land_xy, self._pred_xy)
            decay = float(cmd.cfg.exact_success_decay)
            self._err_acc = decay * self._err_acc + float(err[both].sum())
            self._err_n_acc = decay * self._err_n_acc + float(both.sum())
        self._land_new.zero_()

    def _on_physics_step(self, dt: float) -> None:
        """Physics-substep callback (table_tennis_env.py mechanism): aero wrench + landing scan.

        Asset ``data`` buffers are lazily refreshed against the sim timestamp, so reads here are
        per-substep fresh (the same property table_tennis_env.py's aero callback relies on). The
        wrench is written for the FULL batch every substep (zeros where inactive) so a ball that
        just landed/parked never keeps a stale external force.
        """
        active = (self._mode == _MODE_POST) & ~self._landed
        lin_vel_w = self._ball.data.root_lin_vel_w
        ang_vel_w = self._ball.data.root_ang_vel_w
        force_w = venue_aero_force(lin_vel_w, ang_vel_w, self._mass, self._prm.k_d, self._prm.k_m)
        force_w = force_w * active.unsqueeze(-1)
        # Isaac Lab 2.1 applies this BODY-frame wrench at the link transform origin.  The
        # origin-centred SphereCfg has zero COM offset, so the point is also the ball COM.
        self._force_b[:, 0, :] = quat_rotate_inverse_wxyz(self._ball.data.root_quat_w, force_w)
        self._ball.set_external_force_and_torque(self._force_b, self._torque_b)
        self._ball.write_data_to_sim()

        self._detect_landings(self._env.scene.env_origins)
