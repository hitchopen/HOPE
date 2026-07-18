"""Input / output locations for the no-spin ball-physics fitting pipeline.

Inputs are canonical ball-trajectory CSVs (columns ``t,x,y,z`` in SI units, one
file per flight segment) -- see ``extract_canonical.py`` and ``sample_data/``.

* ``DATA_ROOT`` -- folder holding the canonical trajectory CSVs to fit.
  Defaults to the bundled ``sample_data/`` so the pipeline runs out of the box;
  point ``BALLFIT_DATA_ROOT`` at your own capture to re-fit.
* ``OUT_ROOT``  -- where the stages write their JSON / PNG artifacts.
  Defaults to ``<DATA_ROOT>/analysis``; override with ``BALLFIT_OUT_ROOT``.
"""
import os

_HERE = os.path.dirname(os.path.abspath(__file__))

DATA_ROOT = os.environ.get("BALLFIT_DATA_ROOT", os.path.join(_HERE, "sample_data"))
OUT_ROOT = os.environ.get("BALLFIT_OUT_ROOT", os.path.join(DATA_ROOT, "analysis"))

SEGMENTS = os.path.join(OUT_ROOT, "segments")
FITS = os.path.join(OUT_ROOT, "fits")
FALSIFICATION = os.path.join(OUT_ROOT, "falsification")


def trajectory_files(data_root=None):
    """Sorted list of canonical trajectory CSVs under ``data_root`` (or DATA_ROOT).

    A canonical CSV has a ``t,x,y,z`` header and holds one contiguous capture
    segment (SI units: seconds and metres, world frame from the config).
    """
    import glob
    root = data_root or DATA_ROOT
    files = sorted(glob.glob(os.path.join(root, "*.csv")))
    if not files:
        raise FileNotFoundError(
            f"No trajectory CSVs found under {root!r}. Set BALLFIT_DATA_ROOT to a "
            "folder of canonical t,x,y,z ball-trajectory CSVs (produce them from a "
            "raw capture with extract_canonical.py).")
    return files
