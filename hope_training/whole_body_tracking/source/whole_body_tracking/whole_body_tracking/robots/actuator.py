"""Optional delayed implicit-PD actuator (command latency domain randomization).

An :class:`~isaaclab.actuators.ImplicitActuator` whose position/velocity/effort setpoints are lagged
by a configurable number of physics steps, drawn uniformly in ``[min_delay, max_delay]`` at each
reset. Not used by the default HOPE PingPong task (which uses a plain implicit actuator); provided as
a drop-in for anyone who wants to randomize actuation latency.
"""

from __future__ import annotations

import torch
from collections.abc import Sequence

from isaaclab.actuators import ImplicitActuator, ImplicitActuatorCfg
from isaaclab.utils import DelayBuffer, configclass
from isaaclab.utils.types import ArticulationActions


class DelayedImplicitActuator(ImplicitActuator):
    """Implicit-PD actuator that applies its commands a lagged number of physics steps late."""

    cfg: DelayedImplicitActuatorCfg

    def __init__(self, cfg: DelayedImplicitActuatorCfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        self.positions_delay_buffer = DelayBuffer(cfg.max_delay, self._num_envs, device=self._device)
        self.velocities_delay_buffer = DelayBuffer(cfg.max_delay, self._num_envs, device=self._device)
        self.efforts_delay_buffer = DelayBuffer(cfg.max_delay, self._num_envs, device=self._device)
        self._ALL_INDICES = torch.arange(self._num_envs, dtype=torch.long, device=self._device)

    def reset(self, env_ids: Sequence[int]):
        super().reset(env_ids)
        if env_ids is None or env_ids == slice(None):
            num_envs = self._num_envs
        else:
            num_envs = len(env_ids)
        time_lags = torch.randint(
            low=self.cfg.min_delay,
            high=self.cfg.max_delay + 1,
            size=(num_envs,),
            dtype=torch.int,
            device=self._device,
        )
        self.positions_delay_buffer.set_time_lag(time_lags, env_ids)
        self.velocities_delay_buffer.set_time_lag(time_lags, env_ids)
        self.efforts_delay_buffer.set_time_lag(time_lags, env_ids)
        self.positions_delay_buffer.reset(env_ids)
        self.velocities_delay_buffer.reset(env_ids)
        self.efforts_delay_buffer.reset(env_ids)

    def compute(
        self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor
    ) -> ArticulationActions:
        control_action.joint_positions = self.positions_delay_buffer.compute(control_action.joint_positions)
        control_action.joint_velocities = self.velocities_delay_buffer.compute(control_action.joint_velocities)
        control_action.joint_efforts = self.efforts_delay_buffer.compute(control_action.joint_efforts)
        return super().compute(control_action, joint_pos, joint_vel)


@configclass
class DelayedImplicitActuatorCfg(ImplicitActuatorCfg):
    """Configuration for a delayed implicit-PD actuator."""

    class_type: type = DelayedImplicitActuator

    min_delay: int = 0
    """Minimum number of physics steps the actuator command may be delayed."""

    max_delay: int = 0
    """Maximum number of physics steps the actuator command may be delayed."""
