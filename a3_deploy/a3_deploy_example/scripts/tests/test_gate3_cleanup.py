"""Fault-inject Gate3 cleanup with isolated child processes, never ROS/hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import pp_gate3_drain_flights as drain
import run_gate3 as runner


def flight(shot=1, stamp=10, *, terminal=False):
    row = {"shot_id": shot, "samples": 10, "last_stamp_ns": stamp}
    if terminal:
        row["terminal_event"] = {"kind": "ground_crossing", "stamp_ns": stamp}
    return row


def observe(monitor, now, rows=None, alive=None, launched=(1,), error=None):
    return monitor.observe({"rows": rows if rows is not None else [flight()]}, launched,
                           alive if alive is not None else {"simulator": True, "recorder": True},
                           now, error)


@pytest.mark.parametrize("producer", ["simulator", "recorder"])
def test_dead_producer_fails_after_final_flush_grace(producer):
    monitor = drain.FlightDrain(stale_s=10, dead_grace_s=2)
    alive = {"simulator": True, "recorder": True, producer: False}
    assert observe(monitor, 0, alive=alive)["status"] == "draining"
    result = observe(monitor, 2, alive=alive)
    assert result["status"] == "failed"
    assert result["failure_reason"] == "producer_exited"
    assert result["pending_shot_ids"] == [1]
    assert result["dead_producers"] == [producer]


def test_buffered_terminal_evidence_can_finish_after_simulator_exit():
    monitor = drain.FlightDrain()
    assert observe(monitor, 0, alive={"simulator": False})["status"] == "draining"
    result = observe(monitor, 1, [flight(terminal=True)], {"simulator": False})
    assert result["evidence_complete"]


def test_live_recorder_rewrites_do_not_hide_stopped_telemetry():
    monitor = drain.FlightDrain()
    for second in range(10):
        assert observe(monitor, second)["status"] == "draining"
    result = observe(monitor, 10)
    assert result["status"] == "failed"
    assert result["failure_reason"] == "telemetry_stalled"


def test_other_flights_cannot_hide_one_stalled_flight():
    monitor = drain.FlightDrain()
    for second in range(11):
        result = observe(monitor, second, [flight(1), flight(2, 10 + second)])
    assert result["stale_shot_ids"] == [1]


def test_no_first_sample_and_missing_report_fail_instead_of_hanging():
    monitor = drain.FlightDrain()
    assert observe(monitor, 0, [], error="missing")["status"] == "draining"
    assert observe(monitor, 10, [], error="missing")["status"] == "failed"


def test_truncated_report_or_log_cannot_erase_a_pending_flight():
    monitor = drain.FlightDrain()
    observe(monitor, 0)
    result = observe(monitor, 10, [], launched=(), error="invalid JSON")
    assert result["status"] == "failed"
    assert result["pending_shot_ids"] == [1]


def test_live_flight_can_last_longer_than_any_cleanup_deadline():
    monitor = drain.FlightDrain()
    for second in range(301):
        assert observe(monitor, second, [flight(stamp=second + 10)])["status"] == "draining"
    assert observe(monitor, 301, [flight(stamp=311, terminal=True)])["status"] == "complete"


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf")])
def test_watchdogs_cannot_be_disabled_with_invalid_values(value):
    with pytest.raises(ValueError):
        drain.FlightDrain(stale_s=value)


@pytest.mark.parametrize("contents", ['{', '[]', '{"rows":[null]}',
                                      '{"rows":[{"shot_id":"bad"}]}'])
def test_malformed_report_is_missing_evidence(tmp_path, contents):
    path = tmp_path / "physical.json"
    path.write_text(contents)
    report, error = drain.read_report(path)
    assert error
    assert report == {"rows": []}


@pytest.fixture
def processes():
    children = []
    def spawn(code="import time; time.sleep(60)"):
        process = subprocess.Popen([sys.executable, "-c", code], start_new_session=True)
        children.append(process)
        return process
    yield spawn
    for process in children:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


def test_zombie_is_not_a_live_telemetry_producer(processes):
    process = processes("pass")
    # Do not poll/reap until the liveness check has inspected the zombie.
    deadline = time.monotonic() + 3
    while drain.process_alive(process.pid) and time.monotonic() < deadline:
        time.sleep(.01)
    assert not drain.process_alive(process.pid)
    process.wait(timeout=1)


@pytest.mark.parametrize("failed_producer", ["simulator", "recorder"])
@pytest.mark.parametrize("exit_trap", [False, True])
def test_failed_producer_cleanup_exits_and_archives(tmp_path, processes, failed_producer, exit_trap):
    source, out = tmp_path / "telemetry", tmp_path / "result"
    source.mkdir()
    out.mkdir()
    started = time.time()
    simulator, recorder, unrelated = processes(), processes(), processes()
    victim = simulator if failed_producer == "simulator" else recorder
    victim.kill()
    victim.wait(timeout=3)
    physical = source / "pp_physical_ball_report.json"
    physical.write_text(json.dumps({"rows": [flight()]}))
    ball_log = source / "pp_ball.log"
    ball_log.write_text("serve 1: shot_id=1\n")
    (source / "pp_rally_report.json").write_text('{"certification_pass": true}')
    pids = out / "child_pids.txt"
    pids.write_text(f"{simulator.pid}\n{recorder.pid}\n")
    # Execute the actual production shell functions in managed mode; no broad
    # pkill branch, simulator setup, ROS participant, or shared /tmp telemetry.
    script = (SCRIPTS / "pp_gate3_rally.sh").read_text()
    block = script[script.index("gate3_cleanup() {"):script.index('echo "[g3r] sim up')]
    # Isolate the production report destination as well as all configured paths.
    block = block.replace("/tmp/pp_rally_report.json", str(source / "pp_rally_report.json"))
    ending = "\nexit 0\n" if exit_trap else "\ngate3_cleanup; rc=$?; trap - EXIT; exit $rc\n"
    env = {k: v for k, v in os.environ.items() if not k.startswith(("PP_", "A3_"))}
    env.update(SCRIPT_DIR=str(SCRIPTS), PP_MANAGED_RUN="1", PP_CHILD_PIDS_FILE=str(pids),
               PP_SIM_PID=str(simulator.pid), PP_EVIDENCE_PID=str(recorder.pid),
               PP_PHYSICAL_EVIDENCE_JSON=str(physical), PP_BALL_LOG=str(ball_log),
               PP_STOP_LAUNCHES_FILE=str(source / "stop"), PP_DRAIN_STALE_S="2",
               PP_DRAIN_DEAD_GRACE_S="0.1", PP_DRAIN_REPORT_JSON=str(out / "pp_gate3_drain_report.json"))
    engine = subprocess.Popen(["bash", "-c", block + ending], env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        output, _ = engine.communicate(timeout=6)
        assert engine.returncode == 2, output.decode()
        rc = runner.finish_run(out, {}, engine.returncode, started,
                               runner.archive_results(out, started, source))
        report = json.loads((out / "run.json").read_text())
        assert rc == 2 and report["status"] == "failed"
        assert not report["certification_pass"]
        assert report["flight_drain"]["failure_reason"] == "producer_exited"
        assert report["flight_drain"]["pending_shot_ids"] == [1]
        assert not json.loads((out / "pp_rally_report.json").read_text())["certification_pass"]
        assert (out / physical.name).read_bytes() == physical.read_bytes()
        assert "terminal_event" not in json.loads(physical.read_text())["rows"][0]
        simulator.wait(timeout=2)
        recorder.wait(timeout=2)
        assert unrelated.poll() is None
    finally:
        if engine.poll() is None:
            os.killpg(engine.pid, signal.SIGKILL)
        engine.wait(timeout=2)


def test_outer_runner_bounds_cleanup_after_recorded_failure(tmp_path, processes):
    engine = processes("import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)")
    owned, unrelated = processes(), processes()
    (tmp_path / "child_pids.txt").write_text(f"{owned.pid}\n")
    drain.write_report(tmp_path / "pp_gate3_drain_report.json",
                       {"status": "failed", "evidence_complete": False,
                        "pending_shot_ids": [1], "failure_reason": "producer_exited"})
    started = time.time()
    rc = runner.wait_for_engine(engine, tmp_path, started, cleanup_grace_s=.1)
    assert rc == 2
    assert time.time() - started < 6
    owned.wait(timeout=2)
    assert unrelated.poll() is None


@pytest.mark.parametrize("state,expected", [("complete", 0), ("failed", 2),
                                           ("draining", 2), ("missing", 2)])
def test_final_archive_cannot_pass_with_incomplete_drain(tmp_path, state, expected):
    report = tmp_path / "pp_rally_report.json"
    report.write_text('{"certification_pass": true, "rows": [{"shot_id": 1}]}')
    if state != "missing":
        drain.write_report(tmp_path / "pp_gate3_drain_report.json",
                           {"status": state, "evidence_complete": state == "complete"})
    assert runner.finish_run(tmp_path, {}, 0, time.time(), []) == expected
    receipt = json.loads((tmp_path / "run.json").read_text())
    assert receipt["certification_pass"] is (expected == 0)
    assert json.loads(report.read_text())["rows"] == [{"shot_id": 1}]


def test_stalled_telemetry_exits_with_live_processes(tmp_path, processes):
    simulator, recorder = processes(), processes()
    physical = tmp_path / "physical.json"
    physical.write_text(json.dumps({"rows": [flight()]}))
    log = tmp_path / "ball.log"
    log.write_text("serve 1: shot_id=1")
    result_path = tmp_path / "drain.json"
    env = dict(os.environ, PP_PHYSICAL_EVIDENCE_JSON=str(physical),
               PP_BALL_LOG=str(log), PP_DRAIN_REPORT_JSON=str(result_path),
               PP_STOP_LAUNCHES_FILE=str(tmp_path / "stop"), PP_DRAIN_STALE_S="0.2",
               PP_SIM_PID=str(simulator.pid), PP_EVIDENCE_PID=str(recorder.pid))
    result = subprocess.run([sys.executable, str(SCRIPTS / "pp_gate3_drain_flights.py")],
                            env=env, capture_output=True, timeout=3)
    assert result.returncode == 2, result.stderr
    report = json.loads(result_path.read_text())
    assert report["failure_reason"] == "telemetry_stalled"
    assert report["producer_alive"] == {"simulator": True, "recorder": True}


def test_archive_copy_error_cannot_leave_a_passing_run(tmp_path):
    drain.write_report(tmp_path / "pp_gate3_drain_report.json",
                       {"status": "complete", "evidence_complete": True})
    (tmp_path / "pp_rally_report.json").write_text('{"certification_pass":true}')
    errors = ["pp_ball.log: injected copy failure"]
    assert runner.finish_run(tmp_path, {}, 0, time.time(), errors) == 2
    receipt = json.loads((tmp_path / "run.json").read_text())
    assert receipt["status"] == "failed" and not receipt["certification_pass"]
    assert receipt["archive_errors"] == errors


@pytest.mark.parametrize("spawn_fails", [False, True])
def test_unified_entry_archives_failure_and_finishes_run_receipt(tmp_path, monkeypatch, spawn_fails):
    gear, source, policy = tmp_path / "gear", tmp_path / "telemetry", tmp_path / "policy"
    out = tmp_path / "result"
    paths = {"runner_dist": tmp_path / "dist", "sim_install": tmp_path / "sim",
             "hope_ws": tmp_path / "ws"}
    for path in (paths["runner_dist"] / "a3_deploy_onnx_ref_pingpong",
                 paths["runner_dist"] / "run_a3_pingpong.sh",
                 paths["sim_install"] / "bin/aimrt_main",
                 paths["hope_ws"] / "install/local_setup.bash",
                 policy / "params/deploy.yaml", policy / "exported/policy.onnx"):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("isolated test fixture")
    source.mkdir()
    (gear / "scripts").mkdir(parents=True)
    (gear / "scripts/pp_gate3_hitter_pingpong.sh").write_text('''
python3 - "$PP_DRAIN_REPORT_JSON" "$TEST_TELEMETRY_DIR" <<'PY'
import json,sys
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps({"status":"failed", "evidence_complete":False,
    "failure_reason":"producer_exited", "pending_shot_ids":[1]}))
source=Path(sys.argv[2])
(source/'pp_ball.log').write_text('serve 1: shot_id=1\\n')
(source/'pp_rally_report.json').write_text('{"certification_pass":true}')
PY
exit 2
''')
    monkeypatch.setattr(runner, "GEAR", gear)
    monkeypatch.setattr(runner, "runtime_config", lambda _: {k: str(v) for k, v in paths.items()})
    monkeypatch.setattr(runner, "native_inspect", lambda *_: {"policy_abi": "small_station_112_v1"})
    monkeypatch.setattr(runner, "reject_active_runtime", lambda: None)
    monkeypatch.setattr(runner, "open", lambda *_: open(tmp_path / "test.lock", "a"), raising=False)
    archive = runner.archive_results
    monkeypatch.setattr(runner, "archive_results", lambda output, started: archive(output, started, source))
    monkeypatch.setenv("TEST_TELEMETRY_DIR", str(source))
    if spawn_fails:
        def fail(*args, **kwargs):
            raise OSError("injected launch failure")
        monkeypatch.setattr(runner.subprocess, "Popen", fail)
    args = runner.parser().parse_args(["--policy-dir", str(policy), "--output", str(out)])
    assert runner.execute(args) == 2
    receipt = json.loads((out / "run.json").read_text())
    assert receipt["status"] == "failed"
    assert not receipt["certification_pass"]
    if spawn_fails:
        assert "injected launch failure" in receipt["runtime_error"]
    else:
        assert (out / "pp_ball.log").read_text() == "serve 1: shot_id=1\n"
        assert receipt["flight_drain"]["pending_shot_ids"] == [1]
