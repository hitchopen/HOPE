from pathlib import Path
import importlib.util
import json
import os
import signal
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[2]
HELPER = ROOT / "foxglove/helpers/hope_sim2real_capture.py"
EXPORT = ROOT / "foxglove/helpers/hope_sim2real_export.py"
NATIVE = ROOT / "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def fake_package(tmp_path):
    root = tmp_path / "package"
    (root / "config").mkdir(parents=True)
    cfg = "a3_body_drive_debug_record.iceoryx.yaml"
    (root / "config" / cfg).write_text((NATIVE / "config" / cfg).read_text())
    exe = root / "a3_body_drive_debug_record"
    exe.write_text("#!/usr/bin/env python3\nimport signal,time\nfrom pathlib import Path\n"
                   "def close(*args):\n Path('clean_close').write_text('ok'); raise SystemExit(0)\n"
                   "signal.signal(signal.SIGINT,close)\nsignal.signal(signal.SIGTERM,close)\n"
                   "Path('ready').touch()\nwhile True: time.sleep(.02)\n")
    exe.chmod(0o755)
    return root


def command(root, output, *wrapped, reserve=0):
    return [sys.executable, str(HELPER), "--role", "mdu", "--root", str(root),
            "--output", str(output), "--session-id", "test", "--reserve-bytes", str(reserve), "--", *wrapped]


def test_mdu_recorder_loads_ros_without_modifying_controller_environment(tmp_path):
    sys.path.insert(0, str(HELPER.parent))
    capture = load(HELPER, "capture_environment_test")
    ros = tmp_path / "ros"
    setup = ros / "jazzy/setup.bash"
    setup.parent.mkdir(parents=True)
    setup.write_text('export ROS_DISTRO=jazzy\nexport LD_LIBRARY_PATH="/ros/lib:${LD_LIBRARY_PATH:-}"\n')
    inherited = {"PATH": os.environ["PATH"], "LD_LIBRARY_PATH": "/vendor/lib",
                 "CONTROLLER_ONLY": "unchanged"}
    expected = inherited.copy()
    root = tmp_path / "package"
    env = capture.recorder_environment("mdu", root, environ=inherited, ros_prefix=ros)
    assert inherited == expected
    assert env["LD_LIBRARY_PATH"].split(":") == [str(root), str(root / "lib"), "/ros/lib", "/vendor/lib"]
    assert env["ROS_DISTRO"] == "jazzy" and env["CONTROLLER_ONLY"] == "unchanged"
    # Other roles retain the ROS environment already prepared by their launcher.
    assert "ROS_DISTRO" not in capture.recorder_environment("hdu", root, environ=inherited, ros_prefix=ros)
    # Do not silently run a recorder after a ROS setup error.
    setup.write_text("return 7\n")
    with pytest.raises(subprocess.CalledProcessError):
        capture.recorder_environment("mdu", root, environ=inherited, ros_prefix=ros)


def test_recorder_failure_detail_retains_missing_library(tmp_path):
    sys.path.insert(0, str(HELPER.parent))
    capture = load(HELPER, "capture_error_detail_test")
    (tmp_path / "recorder.log").write_text(
        "[Error][loader] Load dynamic lib failed: libsensor_msgs__rosidl_generator_c.so: not found\n"
        "[Error][plugin] Initialize failed\n[Info] Shutdown complete\n")
    assert "libsensor_msgs__rosidl_generator_c.so" in capture.recorder_error_detail(tmp_path)


def test_wrapper_preserves_exit_and_closes_recorder(tmp_path):
    root = fake_package(tmp_path)
    output = tmp_path / "capture"
    child = ("import time,json;from pathlib import Path;"
             f"p=Path({str(output / 'runner_trace.csv.health.json')!r});"
             "p.write_text(json.dumps({'closed':False,'dropped_bytes':0,'io_error':False}));"
             "time.sleep(.5);"
             "p.write_text(json.dumps({'closed':True,'dropped_bytes':0,'io_error':False}));"
             "raise SystemExit(7)")
    p = subprocess.run(command(root, output, sys.executable, "-c", child), timeout=15)
    assert p.returncode == 7
    assert (root / "clean_close").read_text() == "ok"
    data = json.loads((output / "status.json").read_text())
    assert data["state"] == "CLOSED" and data["recorder_exit_code"] == 0
    assert not data["forced_close"]
    assert data["csv_health"]["runner_trace.csv.health.json"]["closed"]
    assert not data["errors"]
    assert "max_bag_num: 0" in (output / "body_recorder.yaml").read_text()
    assert list((output / "configuration").rglob("*.yaml"))
    before = (output / "status.json").read_bytes()
    assert subprocess.run(command(root, output), capture_output=True).returncode != 0
    assert (output / "status.json").read_bytes() == before


@pytest.mark.parametrize("failure", ["missing_recorder", "disk_reserve"])
def test_capture_failure_does_not_gate_wrapped_control(tmp_path, failure):
    root = fake_package(tmp_path)
    if failure == "missing_recorder":
        (root / "a3_body_drive_debug_record").unlink()
    output = tmp_path / "capture"
    done = tmp_path / "controller_ran"
    child = f"from pathlib import Path;Path({str(done)!r}).write_text('unchanged');raise SystemExit(3)"
    result = subprocess.run(command(root, output, sys.executable, "-c", child,
                                   reserve=2**80 if failure == "disk_reserve" else 0), timeout=15)
    assert result.returncode == 3 and done.read_text() == "unchanged"
    status = json.loads((output / "status.json").read_text())
    # exec through taskset can fail asynchronously, so either startup or exit is recorded.
    assert status["errors"] or status["recorder_exit_code"] != 0


def test_signal_reaches_controller_and_recorder(tmp_path):
    root = fake_package(tmp_path)
    output = tmp_path / "capture"
    controller = tmp_path / "controller.py"
    marker = tmp_path / "controller_closed"
    ready = tmp_path / "controller_ready"
    controller.write_text("import signal,time\nfrom pathlib import Path\n"
        f"def close(*a):\n Path({str(marker)!r}).touch(); raise SystemExit(0)\n"
        "signal.signal(signal.SIGINT,close)\n"
        f"Path({str(ready)!r}).touch()\nwhile True: time.sleep(.02)\n")
    p = subprocess.Popen(command(root, output, sys.executable, str(controller)))
    try:
        deadline = time.monotonic() + 8
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert ready.exists()
        p.send_signal(signal.SIGINT)
        assert p.wait(timeout=8) == 0
        assert marker.exists() and (root / "clean_close").exists()
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()


def test_monitor_disk_failure_does_not_stop_or_restart_controller(tmp_path):
    root = fake_package(tmp_path)
    output = tmp_path / "capture"
    marker = tmp_path / "controller_completed"
    driver = tmp_path / "inject_failure.py"
    driver.write_text("import sys,errno\nfrom pathlib import Path\n"
        f"sys.path.insert(0,{str(HELPER.parent)!r})\nimport hope_sim2real_capture as m\n"
        "original=m.atomic_json\n"
        "def failing(path,data):\n"
        " if data.get('state')=='RECORDING': raise OSError(errno.ENOSPC,'injected disk full')\n"
        " return original(path,data)\n"
        "m.atomic_json=failing\nraise SystemExit(m.main())\n")
    cmd = command(root, output, sys.executable, "-c",
                  f"import time;from pathlib import Path;time.sleep(.5);Path({str(marker)!r}).write_text('finished');raise SystemExit(9)")
    cmd[1] = str(driver)
    result = subprocess.run(cmd, timeout=15)
    assert result.returncode == 9 and marker.read_text() == "finished"
    status = json.loads((output / "status.json").read_text())
    assert any("injected disk full" in error for error in status["errors"])
    assert status["recorder_exit_code"] == 0


def test_export_preserves_native_timestamps_and_marks_missing_streams(tmp_path):
    np = pytest.importorskip("numpy")
    mcap = pytest.importorskip("mcap.writer")
    pytest.importorskip("rosbags")
    conv = load(NATIVE / "scripts/tools/a3_body_drive_debug_convert.py", "capture_test_converter")
    exporter = load(EXPORT, "capture_test_exporter")
    types = conv.get_ros2_typestore()
    session = tmp_path / "session"
    session.mkdir()
    with (session / "body.mcap").open("wb") as f:
        writer = mcap.Writer(f)
        writer.start()
        typename = "joint_msgs/msg/JointState"
        schema = writer.register_schema(typename, "ros2msg", types.generate_msgdef(typename)[0].encode())
        channel = writer.register_channel("/body_drive/waist_joint_state", "cdr", schema)
        for i, seq in enumerate([40, 41, 44]):
            joints = [types.types["joint_msgs/msg/State"](n, seq, .1 + j, .2, .3) for j, n in enumerate(conv.A3_JOINT_NAMES_31[:3])]
            obj = types.types[typename](conv.make_header(types, 10**9 + i * 2000000), joints)
            writer.add_message(channel, 10**18 + i * 2000000, types.serialize_cdr(obj, typename), publish_time=10**18)
        writer.finish()
    output = tmp_path / "export"
    report = exporter.export(session, output, NATIVE / "scripts/tools/a3_body_drive_debug_convert.py")
    topic = report["mcap"]["body.mcap"]["/body_drive/waist_joint_state"]
    assert topic["count"] == 3 and topic["joint_sequence_gaps"] == 6
    data = np.load(output / topic["npz"])
    assert data["header_stamp_ns"].tolist() == [10**9, 1002000000, 1004000000]
    assert data["log_time_ns"].tolist() == [10**18, 10**18 + 2000000, 10**18 + 4000000]
    assert data["position"].shape == (3, 3)
    assert not report["errors"]
    assert report["status"] == "INCOMPLETE_FOR_POLICY_SIM2REAL"


def test_async_csv_preserves_bytes_and_never_overwrites(tmp_path):
    source = tmp_path / "test.cpp"
    source.write_text('''#include "a3_deploy/async_csv_file.hpp"
#include <iomanip>
int main(int argc,char**argv) {
  {a3_deploy::AsyncCsvFile f; f.open(argv[1]); if(!f) return 1;
   f<<std::setprecision(17)<<"tick,value\\n";
   for(int i=0;i<50000;++i) f<<i<<','<<0.12345678901234567<<'\\n';}
  a3_deploy::AsyncCsvFile second; second.open(argv[1]); return second ? 2 : 0;
}''')
    binary = tmp_path / "test"
    subprocess.run(["g++", "-O2", "-std=c++17", "-pthread", "-I", str(NATIVE / "include"),
                    str(source), "-o", str(binary)], check=True)
    output = tmp_path / "samples.csv"
    subprocess.run([str(binary), str(output)], check=True, timeout=10)
    health = json.loads(output.with_suffix(".csv.health.json").read_text())
    assert health["closed"] and not health["io_error"] and health["dropped_bytes"] == 0
    lines = output.read_text().splitlines()
    assert len(lines) == 50001 and lines[-1] == "49999,0.12345678901234566"
    assert health["written_bytes"] == output.stat().st_size


def test_session_storage_survives_tmp_link_removal(tmp_path):
    helper = (ROOT / "foxglove/helpers/hope-lifecycle").read_text()
    function = helper[helper.index("prepare_persistent_role() {"):helper.index("prepare_hdu() {")]
    # Exercise the real shell function against an isolated substitute /tmp.
    function = function.replace("/tmp/hope_real", str(tmp_path / "tmp"))
    script = ('set -euo pipefail\nSESSION_ID=test_session\nCAPTURE_SESSION_ROOT="$1"\n' + function +
              '\nprepare_persistent_role mdu\nprepare_persistent_role mdu\n')
    persistent = tmp_path / "persistent"
    subprocess.run(["bash", "-c", script, "test", str(persistent)], check=True)
    link = tmp_path / "tmp/test_session/mdu"
    assert link.is_symlink()
    (link / "test.data").write_text("retained")
    link.unlink()
    subprocess.run(["bash", "-c", script, "test", str(persistent)], check=True)
    assert (link / "test.data").read_text() == "retained"


def test_laptop_snapshot_keeps_main_natnet_and_selected_physics(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(HELPER.parent))
    capture = load(HELPER, 'capture_snapshot_test')
    root, output, home = tmp_path / 'repo', tmp_path / 'out', tmp_path / 'home'
    output.mkdir()
    relative = 'NatNet2ROS2/src/motion_capture_tracking_interfaces/msg/NamedPose.msg'
    message = root / relative
    message.parent.mkdir(parents=True)
    message.write_text('string name\n')
    selected = home / '.config/hope-foxglove/ball_physics.json'
    selected.parent.mkdir(parents=True)
    selected.write_text('{"chosen":"berkeley"}\n')
    monkeypatch.setattr(Path, 'home', classmethod(lambda cls: home))
    capture.snapshot(root, output, 'laptop')
    assert (output / 'configuration' / relative).read_bytes() == message.read_bytes()
    assert (output / 'configuration/selected_ball_physics.json').read_bytes() == selected.read_bytes()
    assert '/ball/point' in capture.ROS_TOPICS and '/planner/diagnostics' in capture.ROS_TOPICS
