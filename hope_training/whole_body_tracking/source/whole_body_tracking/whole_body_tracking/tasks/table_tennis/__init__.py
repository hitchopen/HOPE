"""Table-tennis match simulation environment (world frame).

A modular Isaac Lab task that simulates ping-pong ball flight / bounce / racket+table contact together
with a humanoid robot, in the canonical world frame. Ball flight uses a no-spin quadratic drag model on
top of PhysX gravity + contacts; all physical constants are sourced from ``configs/ball_physics.yaml``.
Robot-specific environments and Gym registrations live under :mod:`.config` (e.g. ``config/agibot_a3``).
"""

from .config import agibot_a3  # noqa: F401  — runs gym.register(...) on import
