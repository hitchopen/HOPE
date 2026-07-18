"""Package-local training assets.

``ASSET_DIR`` is the directory where the robot asset (URDF/USD + meshes) is expected. The public
repo ships this directory empty (see ``.gitignore``); supply your own Agibot A3 asset and point the
robot config's ``asset_path`` at it. Nothing here touches the filesystem at import time, so the task
imports fine without the asset present — the path is only resolved when an environment is created.
"""

from pathlib import Path

ASSET_DIR = str(Path(__file__).resolve().parent)
