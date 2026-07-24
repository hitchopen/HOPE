"""HOPE whole-body tracking task.

Importing this package registers the Gym environments ``HOPE-PingPong-AgibotA3-v0`` and
``HOPE-PingPong-UnitreeG1-v0``.
"""

from .config import agibot_a3  # noqa: F401  — runs gym.register(...) on import
from .config import unitree_g1  # noqa: F401  — runs gym.register(...) on import
