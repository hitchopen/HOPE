from __future__ import annotations

import importlib.util
import socket
import threading
import time
from pathlib import Path
from typing import Any

import pytest


A3_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_PATH = A3_ROOT / "scripts" / "a3p_gripper_bridge.py"
GRIP_PATH = (
    A3_ROOT / "scripts" / "serve_gripper_presets.py"
).resolve()
SPEC = importlib.util.spec_from_file_location(
    "a3p_gripper_bridge", BRIDGE_PATH
)
assert SPEC is not None and SPEC.loader is not None
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


def test_packaged_runner_loads_vendor_ros_only_for_gripper_bridge() -> None:
    build_script = (A3_ROOT / "scripts" / "build_a3_deploy_pkg.sh").read_text()
    assert "GRIPPER_VENDOR_ROS_PREFIX=/opt/agibot/share/ros2_package/aimrt_protocol_ros2_package" in build_script
    assert "GRIPPER_ROS_PYTHON_SITE=/opt/ros/jazzy/lib/python3.11/site-packages" in build_script
    assert 'PYTHONPATH="${GRIPPER_PYTHONPATH}"' in build_script
    assert 'LD_LIBRARY_PATH="${GRIPPER_LD_LIBRARY_PATH}"' in build_script
    assert "serve gripper ROS 2 Python imports failed" in build_script
    assert "from ros2_plugin_proto.msg import RosMsgWrapper" in build_script
    assert "aimdk.protocol_pb2" not in build_script


def test_packaged_runner_uses_single_build4_full31_serve_timeline() -> None:
    build_script = (A3_ROOT / "scripts" / "build_a3_deploy_pkg.sh").read_text()
    filename = (
        "a3p_op3_serve025_smooth_center_v14.csv"
    )
    motion_root = A3_ROOT / "assets/a3_runtime/serve/motions"
    path = motion_root / filename
    assert path.is_file()
    # Header + five PREAMBLE metadata rows + all 468 original 100 Hz FRAME rows.
    assert len(path.read_text().splitlines()) == 474
    assert filename in build_script
    assert "SERVE_TIMELINE_ARGS=(" in build_script
    assert f'"${{SCRIPT_DIR}}/motions/{filename}"' in build_script
    assert "unsupported packaged serve override" in build_script
    assert "use --serve-timeline PATH for a replacement CSV" in build_script
    assert "SERVE_TIMELINE_OVERRIDE=1" in build_script
    assert '"${SERVE_TIMELINE_ARGS[@]}"' in build_script
    for old_filename in (
        "a3p_op3_spin001_deploy_parity_entry_v1.csv",
        "a3p_op3_spin001_deploy_parity_timed_serve_v1.csv",
        "a3p_op3_spin001_deploy_parity_recovery_v1.csv",
    ):
        assert old_filename not in build_script
    assert "a3p_op3_serve025_manual_timeline_v1.csv" not in build_script
    assert "a3p_op3_serve025_pdstand_to_ready_v1.csv" not in build_script


def test_hand_command_topic_encoder_matches_c89_protobuf_wire_bytes() -> None:
    encoded = bridge.serialize_hand_command_channel(
        7,
        {"cmd": 0, "pos": 800, "force": 20, "method": 1, "vel": 40},
        1.25,
    )
    # Generated from the checked vendor field numbers used by c89e5ab:
    # Header(seq/timestamp/MANUAL), left agi_claw_cmd, right idle oneof.
    assert encoded.hex() == (
        "0a100807120a08011080e59a7718e2092001"
        "12110a0b1a0910a00618142001302812021a00"
    )


def test_hand_command_topic_encoder_preserves_release_and_idle_right_oneof() -> None:
    encoded = bridge.serialize_hand_command_channel(
        7,
        {"cmd": 0, "pos": 2000, "force": 20, "method": 1, "vel": 60},
        1.25,
    )
    assert encoded.hex() == (
        "0a100807120a08011080e59a7718e2092001"
        "12110a0b1a0910d00f18142001303c12021a00"
    )
    assert encoded.endswith(b"\x12\x02\x1a\x00")


def test_grip_publisher_uses_c89_topic_wrapper_contract() -> None:
    class WrapperMessage:
        def __init__(self) -> None:
            self.serialization_type = ""
            self.context: list[str] = []
            self.data: list[bytes] = []

    class Publisher:
        message: WrapperMessage | None = None

        def publish(self, message: WrapperMessage) -> None:
            self.message = message

    publisher = object.__new__(bridge.GripPublisher)
    publisher.wrapper_type = WrapperMessage
    publisher.publisher = Publisher()
    publisher.command_topic = bridge.COMMAND_TOPIC
    publisher.sequence = 0
    dispatch_ns = publisher.publish(
        {"cmd": 0, "pos": 800, "force": 20, "method": 1, "vel": 40}
    )
    assert dispatch_ns > 0
    assert publisher.sequence == 1
    assert publisher.publisher.message is not None
    assert publisher.publisher.message.serialization_type == "pb"
    assert publisher.publisher.message.context == [
        "aimdk.protocol.HandCommandChannel"
    ]
    assert all(len(value) == 1 for value in publisher.publisher.message.data)


def test_grip_publisher_uses_only_the_c89_endpoint() -> None:
    source = BRIDGE_PATH.read_text(encoding="utf-8")
    assert bridge.COMMAND_TOPIC == (
        "/body_drive/hand_joint_command/"
        "pb_3Aaimdk_2Eprotocol_2EHandCommandChannel"
    )
    assert "/motion/control/hand_joint_command" not in source
    assert "COMMAND_TOPICS" not in source


def test_c89_burst_fails_if_topic_subscriber_is_lost(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    publisher = object.__new__(bridge.GripPublisher)
    attempts: list[dict[str, int]] = []

    def publish(config: dict[str, int]) -> int:
        attempts.append(dict(config))
        return 123456

    publisher.publish = publish
    publisher.spin_once = lambda _timeout: None
    matches = iter([1, 0])
    publisher.matched_subscribers = lambda: next(matches)
    monkeypatch.setattr(bridge, "BURST_PERIOD_S", 0.0)
    contract = bridge.load_grip_contract(GRIP_PATH)
    with pytest.raises(
        RuntimeError, match="hand_command_subscriber_lost_during_burst"
    ):
        publisher.publish_burst(contract.grab)
    assert attempts == [contract.grab]


def response_fields(packet: bytes) -> list[str]:
    fields = packet.decode("ascii").split("\t")
    assert len(fields) == 12
    return fields


def wait_until(predicate: Any, timeout_s: float = 2.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert predicate()


def test_contract_is_content_bound_without_importing_grip_runtime() -> None:
    contract = bridge.load_grip_contract(GRIP_PATH)
    assert contract.source_sha256 == bridge.CANONICAL_GRIP_SHA256
    assert contract.grab == {
        "method": 1,
        "pos": 800,
        "force": 20,
        "vel": 40,
        "cmd": 0,
    }
    assert contract.release == {
        "method": 1,
        "pos": 2000,
        "force": 20,
        "vel": 60,
        "cmd": 0,
    }
    with pytest.raises(RuntimeError, match="path_must_be_absolute"):
        bridge.load_grip_contract(Path("grip.py"))


@pytest.mark.parametrize(
    ("packet", "detail"),
    [
        (b"", "request_size"),
        (b"A3P_GRIPPER_V3\t1", "request_field_count"),
        (b"A3P_GRIPPER_V1\t1\tSTATUS", "protocol_version"),
        (b"A3P_GRIPPER_V3\t0\tSTATUS", "request_id"),
        (b"A3P_GRIPPER_V3\t+1\tSTATUS", "request_id"),
        (b"A3P_GRIPPER_V3\t1\tDROP", "command"),
        (b"A3P_GRIPPER_V3\t1\tGRAB\n", "command"),
        (b"\xff\t1\tSTATUS", "request_not_ascii"),
    ],
)
def test_protocol_v3_parser_is_strict(
    packet: bytes, detail: str
) -> None:
    with pytest.raises(bridge.ProtocolError, match=f"^{detail}$"):
        bridge.parse_request(packet)


def test_response_never_claims_physical_confirmation() -> None:
    fields = response_fields(
        bridge.format_response(
            7,
            ok=True,
            command="RELEASE",
            state="RELEASED",
            first_publish_monotonic_ns=123456,
            published_before_ack=1,
            planned_publish_count=bridge.BURST_COUNT,
            matched_subscribers=1,
        )
    )
    assert fields[:9] == [
        bridge.PROTOCOL_VERSION,
        "7",
        "OK",
        "RELEASE",
        "RELEASED",
        "123456",
        "1",
        str(bridge.BURST_COUNT),
        "1",
    ]
    assert fields[9:12] == ["0", "0", "none"]


class FakeBridgePublisher:
    instances: list["FakeBridgePublisher"] = []

    def __init__(self) -> None:
        self.closed = False
        self.published: list[dict[str, int]] = []
        self.command_topic = bridge.COMMAND_TOPIC
        self.instances.append(self)

    @staticmethod
    def wait_for_subscriber(
        _timeout: float, _stop: Any = None
    ) -> int:
        return 1

    @staticmethod
    def spin_once(_timeout: float) -> None:
        return None

    @staticmethod
    def matched_subscribers() -> int:
        return 1

    def publish_burst(
        self, config: dict[str, int], _stop: Any = None
    ) -> tuple[int, int]:
        first = time.monotonic_ns()
        self.published.extend(
            dict(config) for _ in range(bridge.BURST_COUNT)
        )
        return first, bridge.BURST_COUNT

    def publish(self, config: dict[str, int]) -> int:
        self.published.append(dict(config))
        return time.monotonic_ns()

    def close(self) -> None:
        self.closed = True


def start_fake_bridge(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> tuple[Path, Path, threading.Thread, list[BaseException]]:
    tmp_path.chmod(0o700)
    socket_path = (tmp_path / "bridge.sock").resolve()
    ready_file = (tmp_path / "bridge.ready").resolve()
    FakeBridgePublisher.instances = []
    monkeypatch.setattr(bridge, "GripPublisher", FakeBridgePublisher)
    monkeypatch.setattr(bridge.signal, "signal", lambda *_args: None)
    failures: list[BaseException] = []

    def run() -> None:
        try:
            assert (
                bridge.run_bridge(
                    socket_path,
                    ready_file,
                    0.1,
                    bridge.load_grip_contract(GRIP_PATH),
                )
                == 0
            )
        except BaseException as exc:
            failures.append(exc)

    thread = threading.Thread(target=run, name="fake-gripper-bridge")
    thread.start()
    wait_until(ready_file.exists)
    return socket_path, ready_file, thread, failures


def test_bridge_is_motionless_then_accepts_independent_idempotent_edges(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    socket_path, ready_file, thread, failures = start_fake_bridge(
        tmp_path, monkeypatch
    )
    publisher = FakeBridgePublisher.instances[0]
    assert publisher.published == []
    assert ready_file.read_text(encoding="ascii").split("\t")[:4] == [
        "READY",
        bridge.PROTOCOL_VERSION,
        "IDLE",
        "1",
    ]

    client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    client.settimeout(2.0)
    client.connect(str(socket_path))

    def exchange(request_id: int, command: str) -> list[str]:
        client.sendall(
            f"{bridge.PROTOCOL_VERSION}\t{request_id}\t{command}".encode(
                "ascii"
            )
        )
        return response_fields(client.recv(4096))

    assert exchange(1, "STATUS")[2:5] == ["OK", "STATUS", "IDLE"]
    grabbed = exchange(2, "GRAB")
    assert grabbed[2:5] == ["OK", "GRAB", "GRABBED"]
    assert grabbed[6:9] == [
        str(bridge.BURST_COUNT),
        str(bridge.BURST_COUNT),
        "1",
    ]

    # The bridge has no physical hand-state feedback.  All three actuator
    # edges remain independently retryable instead of trusting a stale local
    # state as a command gate.
    opened = exchange(3, "OPEN")
    assert opened[2:5] == ["OK", "OPEN", "OPEN"]
    assert opened[6:9] == [
        str(bridge.BURST_COUNT),
        str(bridge.BURST_COUNT),
        "1",
    ]
    released = exchange(4, "RELEASE")
    assert released[2:5] == ["OK", "RELEASE", "RELEASED"]
    assert released[6:9] == [
        str(bridge.BURST_COUNT),
        str(bridge.BURST_COUNT),
        "1",
    ]
    grabbed = exchange(5, "GRAB")
    assert grabbed[2:5] == ["OK", "GRAB", "GRABBED"]
    assert grabbed[6:9] == [
        str(bridge.BURST_COUNT),
        str(bridge.BURST_COUNT),
        "1",
    ]
    released = exchange(6, "RELEASE")
    assert released[2:5] == ["OK", "RELEASE", "RELEASED"]
    assert released[6:9] == [
        str(bridge.BURST_COUNT),
        str(bridge.BURST_COUNT),
        "1",
    ]

    wait_until(lambda: len(publisher.published) == 25)

    contract = bridge.load_grip_contract(GRIP_PATH)
    assert publisher.published == [
        *([contract.grab] * bridge.BURST_COUNT),
        *([contract.release] * bridge.BURST_COUNT),
        *([contract.release] * bridge.BURST_COUNT),
        *([contract.grab] * bridge.BURST_COUNT),
        *([contract.release] * bridge.BURST_COUNT),
    ]

    shutdown = exchange(7, "SHUTDOWN")
    assert shutdown[2:5] == ["OK", "SHUTDOWN", "STOPPING"]
    client.close()
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert failures == []
    assert publisher.closed
    assert not socket_path.exists()
    assert not ready_file.exists()


def test_release_fails_honestly_if_subscriber_is_lost_during_burst(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        FakeBridgePublisher,
        "matched_subscribers",
        staticmethod(lambda: 0),
    )
    monkeypatch.setattr(
        FakeBridgePublisher,
        "publish_burst",
        lambda _self, _config, _stop=None: (_ for _ in ()).throw(
            RuntimeError("hand_command_subscriber_lost_during_burst")
        ),
    )
    socket_path, ready_file, thread, failures = start_fake_bridge(
        tmp_path, monkeypatch
    )
    publisher = FakeBridgePublisher.instances[0]
    assert ready_file.read_text(encoding="ascii").split("\t")[:4] == [
        "READY",
        bridge.PROTOCOL_VERSION,
        "IDLE",
        "1",
    ]

    client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    client.settimeout(2.0)
    client.connect(str(socket_path))

    def exchange(request_id: int, command: str) -> list[str]:
        client.sendall(
            f"{bridge.PROTOCOL_VERSION}\t{request_id}\t{command}".encode(
                "ascii"
            )
        )
        return response_fields(client.recv(4096))

    assert exchange(1, "STATUS")[2:5] == ["OK", "STATUS", "IDLE"]
    released = exchange(2, "RELEASE")
    assert released[2:5] == [
        "ERR",
        "RELEASE",
        "FAULT",
    ]
    assert released[8] == "0"
    assert released[11] == "hand_command_subscriber_lost_during_burst"
    assert publisher.published == []
    assert exchange(3, "SHUTDOWN")[2:5] == [
        "OK",
        "SHUTDOWN",
        "STOPPING",
    ]
    client.close()
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert failures == []


def test_cli_requires_paired_real_publish_confirmation() -> None:
    with pytest.raises(SystemExit):
        bridge.parse_args(
            [
                "--grip-source",
                str(GRIP_PATH),
                "--socket",
                "/tmp/x",
                "--ready-file",
                "/tmp/y",
            ]
        )
    args = bridge.parse_args(
        [
            "--grip-source",
            str(GRIP_PATH),
            "--socket",
            "/tmp/x",
            "--ready-file",
            "/tmp/y",
            "--allow-publish",
            "--confirm-real-gripper",
        ]
    )
    assert args.allow_publish and args.confirm_real_gripper
