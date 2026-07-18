"""Task implementations for the HOPE Agibot A3 environment.

Importing the sub-packages runs their Gym registrations. ``table_tennis`` provides the shared
no-spin ball / table world; ``tracking`` provides the whole-body policy task
``HOPE-PingPong-AgibotA3-v0`` that is trained, exported, and evaluated by the scripts.
"""

from . import table_tennis  # noqa: F401
from . import tracking  # noqa: F401
