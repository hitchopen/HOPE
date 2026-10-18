"""Validate the distributable field asset without robot or local replay logs."""
import csv
import hashlib
import json
import math
from pathlib import Path

MOTIONS = Path(__file__).resolve().parents[1] / "assets/a3_runtime/serve/motions"
NAME = "a3p_op3_serve025_edge_center_v20"


def test_v20_identity_timing_and_native_joint_schema():
    path = MOTIONS / (NAME + ".csv")
    manifest = json.loads((MOTIONS / (NAME + ".manifest.json")).read_text())
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest["sha256"]
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert [int(r["sequence_index"]) for r in rows] == list(range(len(rows)))
    assert [r["event"] for r in rows if r["row_kind"] == "PREAMBLE"] == [
        "WAIT_LOAD_CLEAR", "GRAB", "WAIT_GRAB_ACK", "WAIT_SECURE_CLEAR", "WAIT_SPACE"]
    frames = [r for r in rows if r["row_kind"] == "FRAME"]
    assert len(frames) == manifest["frames"] == 468
    for prefix in ("q_rad::", "qd_rad_s::", "qdd_rad_s2::"):
        columns = [k for k in frames[0] if k.startswith(prefix)]
        assert len(columns) == 31
        for row in frames:
            assert all(math.isfinite(float(row[k])) for k in columns)
    for i, row in enumerate(frames):
        assert int(row["frame_index"]) == int(row["action_tick"]) == i
        assert math.isclose(float(row["time_s"]), i / manifest["hz"], abs_tol=1e-9)
    events = {r["event"]: int(r["frame_index"]) for r in frames if r["event"] != "NONE"}
    assert events["RELEASE"] == manifest["release_event_frame"] == 48
    assert events["RELEASE"] - manifest["release_lead_frames"] == manifest["release_frame"] == 45
    assert events["COMPLETE"] == manifest["handoff_frame"] == 467
    assert events["RELEASE"] < events["DETACH_TARGET"] < events["NOMINAL_CONTACT"] < events["COMPLETE"]
