"""Probe age is measured at submission, never at callback or DDS receipt."""
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "a3"))
from hope_elink_clock import ClockQualificationGuard
from hope_monitor_core import NtpProbeResult

BOOT = "2d7b263d-3537-4bd1-a4c2-185bc26082b0"
MONITOR = "19e14643cd5a44bea7eabf2ce9488bbd"
OTHER = "b854f129f6e64be185d747c20a51cad1"


def snapshot(**changes):
    value = dict(schema_version=1, clock_boot_id=BOOT, monitor_id=MONITOR,
                 sequence=1, observed_monotonic_s=10., gate_pass=True)
    value.update(changes)
    return json.dumps(value)


def test_original_probe_start_controls_deadline_not_first_delivery():
    guard = ClockQualificationGuard(BOOT)
    assert guard.update(snapshot(), 12.9)
    assert guard.status(12.99) == (True, 10.)
    assert guard.status(13.) == (False, 10.)
    delayed = ClockQualificationGuard(BOOT)
    assert not delayed.update(snapshot(), 13.)
    assert delayed.status(13.) == (False, None)


def test_duplicate_and_reordered_samples_never_renew_or_recover_permission():
    guard = ClockQualificationGuard(BOOT)
    assert guard.update(snapshot(sequence=2), 10.)
    assert not guard.update(snapshot(sequence=2), 11.)
    assert guard.status(11.) == (False, 10.)
    assert not guard.update(snapshot(sequence=1, observed_monotonic_s=11.), 11.)
    assert not guard.update(snapshot(sequence=3, observed_monotonic_s=9.), 11.)
    assert guard.update(snapshot(sequence=3, observed_monotonic_s=11.1), 11.2)
    assert guard.status(11.2) == (True, 11.1)


def test_monitor_replacement_retires_prior_instance_and_requires_newer_probe():
    guard = ClockQualificationGuard(BOOT)
    assert guard.update(snapshot(), 10.)
    assert not guard.update(snapshot(monitor_id=OTHER), 10.1)
    assert guard.update(snapshot(monitor_id=OTHER, observed_monotonic_s=11.), 11.)
    assert not guard.update(snapshot(sequence=100, observed_monotonic_s=12.), 12.)
    assert guard.status(12.)[0] is False
    assert guard.update(snapshot(monitor_id=OTHER, sequence=2,
                                 observed_monotonic_s=12.1), 12.1)


@pytest.mark.parametrize("changes", [
    {"schema_version": True}, {"schema_version": 2}, {"clock_boot_id": ""},
    {"clock_boot_id": OTHER}, {"monitor_id": "not-a-uuid"},
    {"sequence": True}, {"sequence": 1.0}, {"sequence": 0},
    {"sequence": (1 << 53) + 1}, {"gate_pass": 1},
    {"observed_monotonic_s": True}, {"observed_monotonic_s": None},
    {"observed_monotonic_s": -1}, {"observed_monotonic_s": 10.1},
    {"observed_monotonic_s": float("nan")},
    {"observed_monotonic_s": float("inf")}, {"extra": "unknown"},
    {"observed_monotonic_s": 10 ** 1000},
])
def test_bad_snapshot_revokes_qualification(changes):
    guard = ClockQualificationGuard(BOOT)
    assert guard.update(snapshot(), 10.)
    assert not guard.update(snapshot(**changes), 10.)
    assert not guard.status(10.)[0]


@pytest.mark.parametrize("raw", [None, "", "null", "[]", "{}", "{" * 5000,
    snapshot().replace('"schema_version": 1', '"schema_version": 1, "schema_version": 1')])
def test_malformed_wire_never_qualifies(raw):
    guard = ClockQualificationGuard(BOOT)
    assert not guard.update(raw, 10.)
    assert not guard.status(10.)[0]


def test_missing_expected_boot_and_false_probe_fail_closed():
    assert not ClockQualificationGuard("").update(snapshot(), 10.)
    guard = ClockQualificationGuard(BOOT)
    assert guard.update(snapshot(gate_pass=False), 10.)
    assert guard.status(10.) == (False, 10.)


@pytest.mark.parametrize("now", [9., -1., float("nan"), float("inf"), True])
def test_monotonic_rollback_or_invalid_clock_revokes_permission(now):
    guard = ClockQualificationGuard(BOOT)
    assert guard.update(snapshot(), 10.)
    assert not guard.status(now)[0]
    assert not guard.status(10.)[0]


def monitor_fixture():
    """Execute the actual two monitor methods without ROS or chronyc."""
    tree = ast.parse((ROOT / "a3/hope_monitor.py").read_text())
    monitor = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                   and node.name == "HopeMonitor")
    methods = [node for node in monitor.body if isinstance(node, ast.FunctionDef)
               and node.name in {"_poll_ntp", "_submit_ntp_probe"}]
    clock = SimpleNamespace(now=10.)
    namespace = dict(json=json, time=SimpleNamespace(monotonic=lambda: clock.now),
                     NtpProbeResult=NtpProbeResult, Float64=SimpleNamespace,
                     Bool=SimpleNamespace, String=SimpleNamespace, probe_ntp=object())
    exec(compile(ast.Module(body=methods, type_ignores=[]), "monitor_methods", "exec"), namespace)
    fake_type = type("MonitorFixture", (), {name: namespace[name] for name in
                                           ("_poll_ntp", "_submit_ntp_probe")})
    fake = fake_type()
    future = SimpleNamespace(done=lambda: True,
                             result=lambda: NtpProbeResult(gate_pass=True))
    submissions = []
    fake._probe_pool = SimpleNamespace(submit=lambda *args, **kwargs:
                                      submissions.append(fake._ntp_probe_started_monotonic) or future)
    fake.get_parameter = lambda _: SimpleNamespace(value=1.)
    fake.get_logger = lambda: SimpleNamespace(warn=lambda *args, **kwargs: None)
    fake._ntp_future = None
    fake._ntp_probe_started_monotonic = None
    fake._ntp_probe_sequence = 0
    fake._ntp_clock_boot_id = BOOT
    fake._ntp_monitor_id = MONITOR
    publications = {}
    for name in ("offset", "skew", "disp", "qualified", "ntp_gate", "ntp_text",
                 "ntp_qualification"):
        publications[name] = []
        setattr(fake, "pub_" + name,
                SimpleNamespace(publish=publications[name].append))
    return fake, clock, future, submissions, publications


def test_monitor_stamps_before_submit_and_preserves_old_completed_probe_age():
    fake, clock, future, submissions, publications = monitor_fixture()
    fake._poll_ntp()
    assert submissions == [10.]
    assert fake._ntp_probe_sequence == 0
    assert not publications["ntp_qualification"]
    clock.now = 100.
    fake._poll_ntp()
    record = json.loads(publications["ntp_qualification"][0].data)
    assert record == json.loads(snapshot())
    assert fake._ntp_probe_sequence == 1
    assert submissions == [10., 100.]
    assert publications["ntp_gate"][0].data is True  # Legacy display unchanged.
    assert not ClockQualificationGuard(BOOT).update(json.dumps(record), clock.now)
    future.done = lambda: False
    fake._poll_ntp()
    assert len(publications["ntp_qualification"]) == 1


def test_monitor_probe_exception_emits_deny_and_submits_next_probe():
    fake, clock, future, submissions, publications = monitor_fixture()
    fake._poll_ntp()
    def fail():
        raise RuntimeError("probe failed")
    future.result = fail
    clock.now = 11.
    fake._poll_ntp()
    record = json.loads(publications["ntp_qualification"][0].data)
    assert record["gate_pass"] is False
    assert record["observed_monotonic_s"] == 10.
    assert submissions == [10., 11.]


def test_monitor_qualification_does_not_use_display_replay_publisher():
    tree = ast.parse((ROOT / "a3/hope_monitor.py").read_text())
    assignments = [node for node in ast.walk(tree) if isinstance(node, ast.Assign)
                   and any(isinstance(target, ast.Attribute)
                           and target.attr == "pub_ntp_qualification" for target in node.targets)]
    assert len(assignments) == 1
    call = assignments[0].value
    assert call.func.attr == "create_publisher"
    assert call.args[1].value == "/hope/ntp/qualification"
    assert call.args[2].value == 1
