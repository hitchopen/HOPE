"""A later serve and teardown must never erase an earlier physical flight."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pp_gate3_core import PhysicalEvidenceAccumulator
from pp_gate3_drain_flights import pending_flights


def sample(acc, shot, t, z, *, active=True, x=2.0, racket=1, table=1, vz=-4.0):
    acc.ingest(stamp_ns=int(t * 1e9), shot_id=shot, active=active,
               position=(x, -0.7625, z), velocity=(2.0, 0.0, vz),
               racket_contact_count=racket, table_contact_count=table,
               net_contact_count=0)


def test_interleaved_flights_require_each_ground_endpoint():
    acc = PhysicalEvidenceAccumulator([1, 2], min_samples=2, max_sample_gap_s=0.05)
    sample(acc, 1, 1.000, .040)
    sample(acc, 2, 1.004, 1.0)
    sample(acc, 1, 1.008, .008, active=False)
    report = acc.report()
    assert report["rows"][0]["trajectory_complete"]
    assert pending_flights(report, [1, 2]) == [2]
    sample(acc, 2, 1.008, .032)
    sample(acc, 2, 1.012, .016, active=False)
    report = acc.report()
    assert report["trajectory_complete_count"] == 2
    assert report["landing_censored_count"] == 0
    assert pending_flights(report, [1, 2]) == []


def test_old_park_teleport_is_not_a_completed_trajectory():
    acc = PhysicalEvidenceAccumulator([1], min_samples=2)
    sample(acc, 1, 1.0, 1.1)
    sample(acc, 1, 1.004, -10, active=False, x=100, racket=0, table=0)
    report = acc.report()
    assert report["trajectory_complete_count"] == 0
    assert report["landing_censored_count"] == 1
    assert not report["physical_contact_measured"]
    assert pending_flights(report, [1]) == [1]


def test_ground_miss_has_complete_trajectory_without_a_legal_landing():
    acc = PhysicalEvidenceAccumulator([1], min_samples=2)
    sample(acc, 1, 1.0, .032, racket=0, table=0)
    sample(acc, 1, 1.004, .016, active=False, racket=0, table=0)
    report = acc.report()
    assert report["trajectory_complete_count"] == 1
    assert not report["rows"][0]["landing_pass"]
    assert report["landing_censored_count"] == 0


def test_ground_telemetry_gap_remains_visible():
    acc = PhysicalEvidenceAccumulator([1], min_samples=2, max_sample_gap_s=.05)
    sample(acc, 1, 1.0, .032)
    sample(acc, 1, 1.2, .016, active=False)
    assert acc.report()["trajectory_complete_count"] == 0


def test_table_rest_requires_observed_stationary_dwell_not_a_flight_timeout():
    def resting(acc, tick, *, active=True, vx=.001, z=.7802):
        acc.ingest(stamp_ns=tick*4_000_000, shot_id=1, active=active,
                   position=(2.2, -.76, z), velocity=(vx, 0, .03),
                   racket_contact_count=0, table_contact_count=0 if tick==0 else 1,
                   net_contact_count=0)
    acc=PhysicalEvidenceAccumulator([1], min_samples=2)
    for t in range(501): resting(acc,t)
    resting(acc,501,active=False)
    r=acc.report()['rows'][0]
    assert r['trajectory_complete']
    assert r['terminal_event']['kind']=='settled_on_table'
    for duration,vx,z in [(100,.001,.7802),(501,.02,.7802),(501,.001,1.2)]:
        acc=PhysicalEvidenceAccumulator([1], min_samples=2)
        for t in range(duration): resting(acc,t,vx=vx,z=z)
        resting(acc,duration,active=False,vx=vx,z=z)
        assert not acc.report()['rows'][0]['trajectory_complete']


def test_drain_waits_for_launched_balls_even_before_first_sample():
    assert pending_flights({"rows": []}, [1]) == [1]


def test_drain_does_not_cut_off_a_flight_at_thirty_seconds(tmp_path, monkeypatch):
    import json
    import pp_gate3_drain_flights as drain
    now = [0.0]
    report = tmp_path / "pp_physical_ball_report.json"
    report.write_text(json.dumps({"rows": [{"shot_id": 1, "samples": 10}]}))
    (tmp_path / "pp_ball.log").write_text("serve 1: shot_id=1")
    monkeypatch.setattr(drain, "Path", lambda value: tmp_path / Path(value).name)
    monkeypatch.setattr(drain.time, "monotonic", lambda: now[0])

    def advance(seconds):
        now[0] += seconds
        if now[0] >= 31:
            report.write_text(json.dumps({"rows": [{"shot_id": 1, "samples": 10,
                                                    "terminal_event": {"stamp_ns": 31_000_000_000}}]}))
    monkeypatch.setattr(drain.time, "sleep", advance)
    assert drain.main() == 0
    assert now[0] >= 31


def test_table_edge_rest_requires_a_natural_terminal_sample():
    for x, expected in [(2.7393, True), (2.75, False)]:
        acc=PhysicalEvidenceAccumulator([1], min_samples=2)
        for tick in range(510):
            acc.ingest(stamp_ns=tick*4_000_000,shot_id=1,active=True,
                       position=(x,-.375,.7802),velocity=(.00004,.00002,.03),
                       racket_contact_count=0,table_contact_count=0 if tick==0 else 1,
                       net_contact_count=0)
        assert not acc.report()['rows'][0]['trajectory_complete']
        acc.ingest(stamp_ns=510*4_000_000,shot_id=1,active=False,
                   position=(x,-.375,.7802),velocity=(.00004,.00002,.03),
                   racket_contact_count=0,table_contact_count=1,net_contact_count=0)
        row=acc.report()['rows'][0]
        assert row['trajectory_complete'] is expected
        if expected: assert row['terminal_event']['kind']=='settled_on_table'
