"""HOPE whole-body tracking extension for Isaac Lab.

Importing this package registers the Gym environments (via :mod:`whole_body_tracking.tasks`).
"""

# Register the Gym environments on import.
from .tasks import *  # noqa: F401,F403
