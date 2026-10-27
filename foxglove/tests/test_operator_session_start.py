"""Managed START must bind the independent operator relay before hardware."""
import sys
from pathlib import Path
from subprocess import CompletedProcess
import pytest
pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_lifecycle_supervisor import LifecycleBackend, LifecycleFailure
from hope_lifecycle_core import LifecycleConfig

@pytest.mark.parametrize("source", ["vrpn", "motive"])
def test_start_rebinds_exact_runner_session_before_hal(source):
    calls = []
    def run(argv, **kwargs):
        calls.append(argv)
        return CompletedProcess(argv, 0, "", "")
    backend = LifecycleBackend(run=run)
    backend.exchange_assets = lambda *a: {"configured_model": "ppq004", "runtime": "VERIFIED"}
    config = LifecycleConfig(revision=1, mocap_source=source, table_side="P2",
                             vrpn_ip="192.168.100.99", pants_profile="v4")
    session = "model21800_20261027T140000Z"
    backend.start(config, session, lambda event: None, ball_model="ppq004")
    rebind = next(c for c in calls if "reconnect-laptop" in c)
    assert rebind[rebind.index("reconnect-laptop")+1] == session
    assert rebind[-5:] == [source, config.vrpn_ip, config.vrpn_port, config.vrpn_units, "v4"]
    hal = next(c for c in calls if "start-hal" in c)
    runner = next(c for c in calls if "start-runner" in c)
    assert calls.index(rebind) < calls.index(hal) < calls.index(runner)
    assert runner[runner.index("start-runner")+1] == session


def test_rebind_failure_never_starts_hal_and_kernel_remains_independent():
    calls = []
    def run(argv, **kwargs):
        calls.append(argv)
        return CompletedProcess(argv, 1 if "reconnect-laptop" in argv else 0, "", "receiver failed")
    backend = LifecycleBackend(run=run)
    config = LifecycleConfig(revision=1, mocap_source="vrpn")
    with pytest.raises(LifecycleFailure, match="receiver failed"):
        backend.start(config, "model21800_20261027T140000Z", lambda event: None)
    assert not any("start-hal" in c or "start-runner" in c for c in calls)
    calls.clear()
    backend.start(config, "model21800_20261027T140000Z", lambda event: None, mode="KERNEL")
    assert not any("reconnect-laptop" in c for c in calls)
    assert any("start-runner" in c for c in calls)
