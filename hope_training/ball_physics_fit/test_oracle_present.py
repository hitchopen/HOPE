"""The venue-data oracle must be present for physics validation — FAIL, never skip.

The old repo test (test_ball_physics_vs_record.py on branch ball-physics-realistic)
silently skipped when $RECORD_DIR was absent, which let unvalidated constants ride
along unnoticed. This test is the loud replacement: on machines without the venue
dataset, set BALLFIT_DATA_ROOT or accept the failure as a signal, do not skip.
"""
import json
import os

import paths


def test_oracle_present():
    paths.require_oracle()


def test_fits_present_and_sane():
    fp = os.path.join(paths.FITS, "stage2_fits_all.json")
    assert os.path.exists(fp), (
        f"stage2_fits_all.json missing under {paths.FITS!r} — run stage2_fits.py")
    F = json.load(open(fp))
    assert 0.05 < F["kd"]["kd"] < 0.25, F["kd"]["kd"]
    assert 0.002 < F["km"]["km"] < 0.008, F["km"]["km"]
    assert 0.80 < F["table_e"]["e_const"] < 1.00, F["table_e"]["e_const"]
    assert 0.30 < F["paddle"]["e_eff"] < 0.95, F["paddle"]["e_eff"]


if __name__ == "__main__":
    test_oracle_present()
    test_fits_present_and_sane()
    print("oracle + fits present and sane")
