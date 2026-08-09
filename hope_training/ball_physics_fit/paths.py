"""Data-root resolution for the ball-physics fitting pipeline.

Set BALLFIT_DATA_ROOT to the venue recording folder (the one holding the
per-take C3D exports and the derived analysis/ tree). The env var is REQUIRED:
the recordings are venue-specific and are not shipped with the repository.
"""
import os

DATA_ROOT = os.environ.get("BALLFIT_DATA_ROOT", "")
if not DATA_ROOT:
    raise SystemExit(
        "BALLFIT_DATA_ROOT is not set; point it at your venue recording folder "
        "(the directory holding the per-take C3D exports and the analysis/ tree).")
ANALYSIS = os.path.join(DATA_ROOT, "analysis")
EXTRACTED = os.path.join(ANALYSIS, "extracted")
SEGMENTS = os.path.join(ANALYSIS, "segments")
FITS = os.path.join(ANALYSIS, "fits")
FALSIFICATION = os.path.join(ANALYSIS, "falsification")


def require_oracle():
    """Fail LOUDLY (never skip) when the extracted venue oracle is missing."""
    if not os.path.isdir(EXTRACTED) or not any(
            f.endswith(".npz") for f in os.listdir(EXTRACTED)):
        raise FileNotFoundError(
            f"Ball-physics oracle not found under {EXTRACTED!r}. "
            "Set BALLFIT_DATA_ROOT to the venue recording folder (containing "
            "analysis/extracted/*.npz produced by extract_canonical.py), or run "
            "extract_canonical.py on the raw C3D takes first. Refusing to skip: "
            "physics constants must be validated against recorded data.")
