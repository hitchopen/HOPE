#!/usr/bin/env python3
"""Isolated native-operations worker fixture: no SSH, vendor or robot processes."""
import os
import json
from pathlib import Path
import time

import hope_lifecycle_supervisor as lifecycle
import hope_monitor_core
from hope_operations_native_state import RuntimeRelay
import rclpy
from rclpy.node import Node
from hope_elink_health import PassiveSnapshot, ProcessIdentity
from hope_lifecycle_core import LifecycleConfig, save_config_atomic


def forbidden_ros(*_args, **_kwargs):
    raise AssertionError("orchestration worker must never initialize ROS or construct a Node")


reference = os.environ.get("HOPE_OPERATIONS_TEST_REFERENCE") == "1"
split = os.environ.get("HOPE_OPERATIONS_TEST_SPLIT") == "1"
if not (reference or split):
    rclpy.init = forbidden_ros
    rclpy.create_node = forbidden_ros
    Node.__init__ = forbidden_ros

root = Path(os.environ["HOPE_OPERATIONS_TEST_ROOT"])
root.mkdir(parents=True, exist_ok=True)
# Change a fixture value, never a real adapter/DHCP lease, to exercise the
# production reconnect path across a real C++/Python service boundary.
hope_monitor_core.read_ipv4_interface_signature = lambda: (
    (root / "network_signature").read_text() if (root / "network_signature").exists() else "initial")
lifecycle.CONFIG_PATH = root / "config.json"
save_config_atomic(lifecycle.CONFIG_PATH, LifecycleConfig(
    laptop_wifi_ip="192.168.8.132", hdu_wifi_ip="192.168.8.182",
    mdu_internal_ip="10.42.10.12", revision=1))
lock = root / "hardware.lock"
lock.touch()
original_lock = lifecycle.try_acquire_hardware_operation_lock
lifecycle.try_acquire_hardware_operation_lock = lambda: original_lock(lock)


class Backend:
    def start(self, config, session, event, *args, **kwargs):
        kwargs["before_mutation"]()
        (root / "session").write_text(session)
        with (root / "actions").open("a") as stream:
            stream.write("START\n")
        time.sleep(.3)

    def kill_all_and_collect(self, *args):
        with (root / "actions").open("a") as stream:
            stream.write("STOP\n")
        while (root / "hold_stop").exists():
            time.sleep(.01)
        return "ISOLATED"

    def observe_elink(self, _config):
        boot = "12345678-1234-1234-1234-123456789abc"
        return PassiveSnapshot(boot, (ProcessIdentity(boot, 101, 200),),
                               (ProcessIdentity(boot, 102, 300),), True)

    def audit_elink(self, _config):
        return {}

    def reconnect_mocap(self, _config):
        pass

    def exchange_recording(self, *args):
        return {"requested": "", "workers": []}

    def exchange_assets(self, _host, _config, request):
        if request["op"] == "read_calibration" and (root / "hold_asset").exists():
            (root / "asset_entered").touch()
            deadline = time.monotonic() + 6.
            while (root / "hold_asset").exists() and time.monotonic() < deadline:
                time.sleep(.01)
            return {"calibration": None}
        raise ValueError("isolated test has no asset endpoint")


lifecycle.LifecycleBackend = Backend
original_field = lifecycle.FieldOperations
def field(*args, **kwargs):
    result = original_field(*args, **kwargs)
    result.runner_selection = lambda _config: ("OPTITRACK", "")
    return result
lifecycle.FieldOperations = field

# Observe the real player callbacks without adding a ROS participant. These
# hooks exist only in this isolated fixture, never in installed worker code.
player_stats = {"counts": {}}
original_player = lifecycle.LifecycleModule._on_player_position
def on_player(self, player, message):
    original_player(self, player, message)
    player_stats["counts"][player] = player_stats["counts"].get(player, 0) + 1

original_init = lifecycle.LifecycleModule.__init__
def init(self, *args, **kwargs):
    original_init(self, *args, **kwargs)
    if (root / "hold_startup").exists():
        (root / "startup_waiting").touch()
        while (root / "hold_startup").exists():
            time.sleep(.01)
        if (root / "fail_startup").exists():
            raise RuntimeError("isolated initialization failure")
    def stats():
        with self._lock:
            player_stats["owner"] = dict(state=self._state, session=self._session_id,
                                         hardware_lock=self._hardware_operation_lock is not None)
        (root / "player_stats.json").write_text(json.dumps(player_stats))
    self.create_timer(.1, stats)
    def fail_control_timer():
        if (root / "fail_control_timer").exists():
            raise RuntimeError("isolated control timer failure")
    self.create_timer(.05, fail_control_timer)

lifecycle.LifecycleModule._on_player_position = on_player
lifecycle.LifecycleModule.__init__ = init

original_runtime = RuntimeRelay._runtime_snapshot
def runtime_snapshot(self):
    if (root / "fail_observer_snapshot").exists():
        raise RuntimeError("isolated observer snapshot failure")
    return original_runtime(self)
RuntimeRelay._runtime_snapshot = runtime_snapshot

if split:
    lifecycle.main()
elif reference:
    import hope_operations
    hope_operations.main()
else:
    import hope_operations_worker
    hope_operations_worker.main()
