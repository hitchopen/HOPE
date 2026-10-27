"""Jointly satisfy actual HOLD geometry and station settling without delaying a ball.

This is a PPO state reward, not a gradient on consecutive policy outputs.  The
old dense pose score remains available for approach; a good limb cannot improve
the additional score while a different constraint is the worst violation.
"""
import math

import torch


CRITERIA = ("joints", "waist_pitch", "torso", "feet", "width", "height", "hands",
            "station", "speed", "heading", "tilt", "joint_speed", "support")


def readiness_ratios(error, torso_error, foot_error, width_error, height_error,
                     hand_error, distance, speed, base_rpy, joint_speed, loaded,
                     waist_pitch_column):
    """Dimensionless physical errors; every component must be <= 1 to settle.

    Pose/station tolerances follow the command-station Gate3 supplement.  Joint
    speed and loaded feet additionally exclude a fast pass through the pose.
    No q_des-q or requested motor torque enters this definition.
    """
    tilt = torch.acos((base_rpy[:, 0].cos()*base_rpy[:, 1].cos()).clamp(-1, 1))
    return torch.stack((
        error.abs().amax(-1)/math.radians(5),
        error[:, waist_pitch_column].abs()/math.radians(3),
        torso_error[:, :2].abs().amax(-1)/math.radians(3),
        foot_error.abs().flatten(1).amax(-1)/math.radians(5),
        width_error.abs()/.02, height_error.abs()/.02,
        hand_error.abs().amax(-1)/.03,
        distance/.05, speed/.12, base_rpy[:, 2].abs()/math.radians(5),
        tilt/math.radians(12), joint_speed.abs().amax(-1)/.5,
        torch.where(loaded, torch.zeros_like(speed), torch.full_like(speed, 2.)),
    ), -1)


def weakest_score(ratios):
    # Rational tail preserves a signal outside tolerance; multiplying 13 small
    # kernels would make ordinary recovery states almost indistinguishable.
    worst = ratios.amax(-1)
    return 1/(1+worst.square())


class HoldDwell:
    """Contiguous physical dwell, reset on episode, station or phase changes."""

    def __init__(self, reference):
        self.elapsed = torch.zeros_like(reference)
        self.was_ready = torch.zeros_like(reference, dtype=torch.bool)
        self.episode_step = torch.full_like(reference, -1, dtype=torch.long)
        self.station = None
        self.step = None

    def update(self, step, episode_step, station, ready, dt):
        if dt <= 0:
            raise ValueError("HOLD dwell requires a positive control timestep")
        if self.step == step:
            return self.elapsed  # diagnostics must not advance the reward clock
        reset = episode_step <= self.episode_step
        if self.station is not None:
            reset |= (station-self.station).abs().amax(-1) > 1e-6
        consecutive = self.step is not None and step == self.step+1
        continuing = ready & self.was_ready & ~reset & consecutive
        self.elapsed = torch.where(continuing, self.elapsed+dt, torch.zeros_like(self.elapsed))
        self.was_ready = ready.detach().clone()
        self.episode_step = episode_step.detach().clone()
        self.station = station.detach().clone()
        self.step = step
        return self.elapsed
