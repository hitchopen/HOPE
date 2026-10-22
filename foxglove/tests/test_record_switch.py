from pathlib import Path
import csv
import importlib.util
import json
import os
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
INCLUDE = ROOT / "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include"


def wait(predicate, seconds=8):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.03)
    assert predicate()


def test_switch_closes_rotates_both_csv_writers_without_stopping_controller(tmp_path):
    source = tmp_path / "probe.cpp"
    source.write_text('''#include "a3_deploy/async_csv_file.hpp"
#include "hope_planner_cpp/audit_logger.hpp"
#include <chrono>
#include <thread>
int main(int argc,char**argv) {
  a3_deploy::AsyncCsvFile out; out.open(std::string(argv[1])+"/runner_trace.csv");
  out<<"tick,value\\n"; out.flush();
  hope_planner_cpp::AuditLogger planner(std::string(argv[1])+"/planner_audit.csv","tick,value");
  for(int i=0;i<1500;++i) {
    out<<i<<','<<i*.01<<'\\n'; out.flush();
    planner.enqueue(std::to_string(i)+",42");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}''')
    planner = ROOT / "hope_ws/src/hope_planner_cpp"
    binary = tmp_path / "probe"
    subprocess.run(["g++", "-O2", "-std=c++17", "-pthread", "-I", str(INCLUDE),
                    "-I", str(planner / "include"), str(source), str(planner / "src/audit_logger.cpp"),
                    "-o", str(binary)], check=True)
    control = tmp_path / "control"
    control.write_text("")
    p = subprocess.Popen([str(binary), str(tmp_path)], env={**os.environ, "HOPE_RECORD_CONTROL_FILE": str(control)})
    try:
        time.sleep(.15)
        assert not (tmp_path / "recordings").exists()
        captures = []
        for name in ["record_first", "record_second"]:
            control.write_text(name)
            directory = tmp_path / "recordings" / name
            wait(lambda: (directory / "runner_trace.csv").exists() and (directory / "planner_audit.csv").exists())
            time.sleep(.2)
            control.write_text("")
            def closed():
                try:
                    return all(json.loads((directory / (n + ".health.json")).read_text())["closed"]
                               for n in ["runner_trace.csv", "planner_audit.csv"])
                except (OSError, ValueError):
                    return False
            wait(closed)
            before = {x.name: x.read_bytes() for x in directory.iterdir()}
            time.sleep(.15)
            assert before == {x.name: x.read_bytes() for x in directory.iterdir()}
            assert p.poll() is None
            for filename in ["runner_trace.csv", "planner_audit.csv"]:
                with (directory / filename).open() as f:
                    rows = list(csv.DictReader(f))
                assert len(rows) > 10 and all(set(row) == {"tick", "value"} and None not in row.values() for row in rows)
                health = json.loads((directory / (filename + ".health.json")).read_text())
                assert health["dropped_bytes"] == 0 and not health["io_error"]
            captures.append(directory)
        with (captures[0] / "runner_trace.csv").open() as f:
            a = list(csv.DictReader(f))
        with (captures[1] / "runner_trace.csv").open() as f:
            b = list(csv.DictReader(f))
        assert int(b[0]["tick"]) > int(a[-1]["tick"])
        assert p.wait(timeout=10) == 0
    finally:
        if p.poll() is None:
            p.kill(); p.wait()


def test_distributed_switch_requires_all_roles_and_off_acknowledgements():
    path = ROOT / "foxglove/a3/hope_distributed_recording.py"
    spec = importlib.util.spec_from_file_location("switch", path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    intents = {}
    ready = {"laptop": True, "hdu": False, "mdu": True}
    def exchange(role, config, request):
        if request["op"] == "set": intents[role] = request["segment"]
        token = intents.get(role, "")
        return {"requested": token, "workers": [{"role": role, "state": "ON" if token and ready[role] else "OFF",
                                                   "segment": token, "errors": []}]}
    recorder = module.DistributedRecorder(exchange, lambda: None)
    assert recorder.start()["state"] == "STARTING"
    time.sleep(.1)
    assert recorder.status()["state"] != "ON"
    ready["hdu"] = True
    wait(lambda: recorder.status()["state"] == "ON")
    first = recorder.status()["output"]
    assert recorder.stop()["state"] == "OFF"
    assert set(intents.values()) == {""}
    recorder.start()
    wait(lambda: recorder.status()["state"] == "ON")
    assert recorder.status()["output"] != first
    recorder.stop()

    # Existing Kernel mode has no Laptop/Planner lifecycle. Its Record switch
    # must still close MDU data without trying to connect to absent peers.
    visited = []
    def kernel_exchange(role, config, request):
        visited.append(role)
        return exchange(role, config, request)
    kernel = module.DistributedRecorder(kernel_exchange, lambda: None, roles=lambda: {"mdu"})
    kernel.start()
    wait(lambda: kernel.status()["state"] == "ON")
    assert kernel.stop()["state"] == "OFF"
    assert set(visited) == {"mdu"}


def test_capture_switch_stops_only_recorder_and_creates_new_segment(tmp_path):
    from test_sim2real_capture import fake_package, command
    import signal
    root = fake_package(tmp_path)
    output = tmp_path / "capture"
    state = tmp_path / "switch"
    env = {**os.environ, "HOPE_RECORD_STATE_ROOT": str(state)}
    marker = tmp_path / "controller_closed"
    controller = tmp_path / "controller.py"
    controller.write_text("import signal,time\nfrom pathlib import Path\n"
        f"def stop(*args):\n Path({str(marker)!r}).touch(); raise SystemExit(0)\n"
        "signal.signal(signal.SIGINT,stop)\nwhile True:time.sleep(.02)\n")
    cmd = command(root, output, sys.executable, str(controller)); cmd.insert(2, "--controlled")
    p = subprocess.Popen(cmd, env=env)
    def status():
        try: return json.loads((output / "monitor.json").read_text())["state"]
        except (OSError, ValueError): return "WAIT"
    try:
        wait(lambda: status() == "OFF")
        assert not (output / "recordings").exists()
        for segment in ["record_one", "record_two"]:
            (state / "recording.request").write_text(segment)
            wait(lambda: status() == "ON")
            (state / "recording.request").write_text("")
            wait(lambda: status() == "OFF")
            report = json.loads((output / "recordings" / segment / "capture/status.json").read_text())
            assert report["state"] == "CLOSED" and report["recorder_exit_code"] == 0
            assert not report["forced_close"] and not report["errors"]
            assert p.poll() is None and not marker.exists()
        p.send_signal(signal.SIGINT)
        assert p.wait(timeout=10) == 0 and marker.exists()
    finally:
        if p.poll() is None: p.kill(); p.wait()


def test_loss_belongs_to_one_recording_and_real_loss_remains_an_error(tmp_path):
    source = tmp_path / "counter.cpp"
    source.write_text(r'''#include "a3_deploy/recording_file.hpp"
#include <fstream>
#include <thread>
int main(int argc,char**argv) {
  const std::string root=argv[1], control=root+"/control";
  {std::ofstream f(control);}
  hope_recording::RecordingFile out(root+"/trace.csv",control,"tick,value\n");
  out.Dropped(616188); out.Poll(); // loss before the first recording
  auto change=[&](const char* name) {
    {std::ofstream f(control);f<<name;}
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    return out.Poll();
  };
  auto g=change("record_clean"); out.Write("1,42\n",g); change("");
  g=change("record_lossy"); out.Write("2,42\n",g);
  out.Dropped(616288); change("");
  std::filesystem::copy_file(root+"/trace.csv.recording.json",root+"/lossy_status.json");
  out.Dropped(716288); // subsequent OFF traffic must not alter a closed take
  g=change("record_recovered"); out.Write("3,42\n",g); change("");
}''')
    binary = tmp_path / "counter"
    subprocess.run(["g++", "-O2", "-std=c++17", "-pthread", "-I", str(INCLUDE),
                    str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary), str(tmp_path)], check=True)
    for segment, dropped in [("clean", 0), ("lossy", 100), ("recovered", 0)]:
        directory = tmp_path / "recordings" / ("record_" + segment)
        health = json.loads((directory / "trace.csv.health.json").read_text())
        assert health["closed"] and health["rows"] == 1
        assert health["dropped_bytes"] == dropped
    assert json.loads((tmp_path / "lossy_status.json").read_text())["state"] == "ERROR"
    status = json.loads((tmp_path / "trace.csv.recording.json").read_text())
    assert status["state"] == "OFF" and status["dropped_bytes"] == 0


def test_off_flood_does_not_fill_csv_queue_or_poison_next_take(tmp_path):
    source = tmp_path / "off_flood.cpp"
    source.write_text(r'''#include "a3_deploy/async_csv_file.hpp"
#include <fstream>
int main(int argc,char**argv) {
  const std::string root=argv[1], control=root+"/control";
  {std::ofstream f(control);}
  a3_deploy::AsyncCsvFile out; out.open(root+"/trace.csv");
  out<<"tick,payload\n"; out.flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  const std::string payload(8000,'a');
  for(int i=0;i<50000;++i) {out<<i<<','<<payload<<'\n';out.flush();}
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  {std::ofstream f(control);f<<"record_after_off";}
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  for(int i=0;i<30;++i) {
    out<<i<<",captured\n";out.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  {std::ofstream f(control);}
  std::this_thread::sleep_for(std::chrono::milliseconds(120));out.close();
}''')
    binary = tmp_path / "off_flood"
    subprocess.run(["g++", "-O2", "-std=c++17", "-pthread", "-I", str(INCLUDE),
                    str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary), str(tmp_path)], check=True,
                   env={**os.environ, "HOPE_RECORD_CONTROL_FILE": str(tmp_path / "control")}, timeout=15)
    directory = tmp_path / "recordings/record_after_off"
    with (directory / "trace.csv").open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) >= 28 and all(row["payload"] == "captured" for row in rows)
    health = json.loads((directory / "trace.csv.health.json").read_text())
    assert health["closed"] and health["dropped_bytes"] == 0 and not health["io_error"]
    assert json.loads((tmp_path / "trace.csv.recording.json").read_text())["state"] == "OFF"
