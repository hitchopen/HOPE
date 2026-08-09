"""Table-tennis environment that adds ball aerodynamics on top of the manager-based RL env.

Everything except aerodynamics is handled by the standard :class:`~isaaclab.envs.ManagerBasedRLEnv`
machinery configured in :mod:`.table_tennis_env_cfg`. PhysX does not model air drag, so this subclass
registers a **physics-step callback** that, every physics substep, reads the ball velocity, computes the
aerodynamic wrench (:func:`.ball.compute_aero_wrench`) and writes it to the ball as an external force.

Using a physics callback (rather than overriding ``step()``) keeps this robust across Isaac Lab versions
and applies the force at the full physics rate (here 400 Hz) for accurate flight. If the callback cannot
be registered for any reason, the environment still runs — the ball just flies on PhysX gravity +
contacts alone, which is a valid (drag-free) scene.
"""

from __future__ import annotations

import torch

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.math import quat_rotate_inverse

from . import geometry
from .ball import compute_aero_wrench
from .table_tennis_env_cfg import TableTennisEnvCfg


class TableTennisEnv(ManagerBasedRLEnv):
    """Manager-based table-tennis env with a per-substep ball aerodynamic force field."""

    cfg: TableTennisEnvCfg

    def __init__(self, cfg: TableTennisEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._aero_active = False
        self._setup_ball_aerodynamics()

    def _setup_ball_aerodynamics(self) -> None:
        self._ball = self.scene["ball"]
        self._aero_cfg = self.cfg.ball_aerodynamics
        self._ball_mass = float(geometry.BALL_MASS)
        # Reusable zeroed external-wrench buffers: (num_envs, num_bodies=1, 3).
        self._aero_force = torch.zeros(self.num_envs, 1, 3, device=self.device)
        self._aero_torque = torch.zeros(self.num_envs, 1, 3, device=self.device)

        if not self._aero_cfg.enabled:
            return
        try:
            # isaaclab.sim.SimulationContext inherits add_physics_callback from the Isaac Sim core
            # SimulationContext; the callback fires once per physics step with the step size.
            self.sim.add_physics_callback("hope_ball_aerodynamics", self._apply_ball_aerodynamics)
            self._aero_active = True
        except Exception as exc:  # pragma: no cover - defensive: never block the sim on aero setup
            import omni.log

            omni.log.warn(
                f"[TableTennisEnv] could not register the ball aerodynamics physics callback "
                f"({exc!r}); the ball will fly on PhysX gravity + contacts only."
            )

    def _apply_ball_aerodynamics(self, dt: float) -> None:
        """Physics-step callback: apply the aerodynamic wrench to the ball (world frame -> body frame)."""
        lin_vel_w = self._ball.data.root_lin_vel_w
        ang_vel_w = self._ball.data.root_ang_vel_w
        force_w, torque_w = compute_aero_wrench(lin_vel_w, ang_vel_w, self._ball_mass, self._aero_cfg)

        # set_external_force_and_torque applies the wrench in the body frame; rotate the world-frame
        # wrench into the ball body frame so the net effect is the intended world-frame force.
        quat_w = self._ball.data.root_quat_w
        self._aero_force[:, 0, :] = quat_rotate_inverse(quat_w, force_w)
        self._aero_torque[:, 0, :] = quat_rotate_inverse(quat_w, torque_w)

        self._ball.set_external_force_and_torque(self._aero_force, self._aero_torque)
        self._ball.write_data_to_sim()
