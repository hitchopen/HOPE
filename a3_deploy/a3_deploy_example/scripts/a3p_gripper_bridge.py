#!/usr/bin/env python3
"""Persistent A3P left-gripper topic bridge for the ping-pong Runner.

OPEN/GRAB/RELEASE use the hardware-proven ``c89e5ab`` path exactly: publish an
``aimdk.protocol.HandCommandChannel`` protobuf on the body-drive hand command
topic, command the left ``agi_claw_cmd`` oneof, select an idle right-claw oneof,
and send five copies 100 ms apart.  The robot image has no generated
``aimdk.protocol_pb2`` Python package, so the small frozen message is encoded
locally using its checked vendor proto field numbers.  The resulting wire bytes
are the same bytes produced by the original ``HandCommandChannel`` class.

Startup is motionless and enters IDLE. OPEN, GRAB and RELEASE are independent,
idempotent allowlisted edges; their receipts are transport telemetry and never
gate the full-body Serve program. No receipt claims finger motion, ball grasp,
or physical release because this firmware provides no usable hand feedback.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import os
import re
import select
import signal
import socket
import stat
import struct
import sys
import time
from pathlib import Path
from typing import Any, Callable, NamedTuple


PROTOCOL_VERSION = "A3P_GRIPPER_V3"
COMMAND_TOPIC = (
    "/body_drive/hand_joint_command/"
    "pb_3Aaimdk_2Eprotocol_2EHandCommandChannel"
)
CANONICAL_GRIP_SHA256 = "778bd928c424abe7cea50a87ea18a6561e9ac13ff9ab81e3747209e911027e4c"
BURST_COUNT = 5
BURST_PERIOD_S = 0.1
MAX_PACKET_BYTES = 512
ALLOWED_COMMANDS = frozenset(
    {"STATUS", "OPEN", "GRAB", "RELEASE", "SHUTDOWN"}
)
ALLOWED_STATES = frozenset(
    {"IDLE", "OPEN", "GRABBED", "RELEASED", "STOPPING", "FAULT"}
)
_DETAIL_RE = re.compile(r"[a-z0-9_.:-]+")


class ProtocolError(ValueError):
    """A malformed, stale, or disallowed private IPC request."""


class GripContract(NamedTuple):
    source_path: Path
    source_sha256: str
    grab: dict[str, int]
    release: dict[str, int]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _literal_assignment(tree: ast.Module, name: str) -> Any:
    for statement in tree.body:
        if not isinstance(statement, (ast.Assign, ast.AnnAssign)):
            continue
        targets = statement.targets if isinstance(statement, ast.Assign) else [statement.target]
        if not any(isinstance(target, ast.Name) and target.id == name for target in targets):
            continue
        value = statement.value
        if value is None:
            break
        # The canonical source spells presets as dict(method=..., ...).
        # Convert those calls to literal dictionaries without executing the
        # module (which imports ROS and is therefore unsafe for offline checks).
        class DictCallToLiteral(ast.NodeTransformer):
            def visit_Call(self, node: ast.Call) -> ast.AST:
                self.generic_visit(node)
                if (
                    isinstance(node.func, ast.Name)
                    and node.func.id == "dict"
                    and not node.args
                    and all(keyword.arg is not None for keyword in node.keywords)
                ):
                    return ast.Dict(
                        keys=[ast.Constant(keyword.arg) for keyword in node.keywords],
                        values=[keyword.value for keyword in node.keywords],
                    )
                return node

        return ast.literal_eval(ast.fix_missing_locations(DictCallToLiteral().visit(value)))
    raise RuntimeError(f"canonical_grip_missing_{name.lower()}")


def load_grip_contract(path: Path) -> GripContract:
    """Load and content-bind the latest Yikang/Catrunaround grip presets."""

    if not path.is_absolute():
        raise RuntimeError("canonical_grip_path_must_be_absolute")
    try:
        metadata = path.lstat()
    except FileNotFoundError as exc:
        raise RuntimeError("canonical_grip_missing") from exc
    if not stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise RuntimeError("canonical_grip_not_regular")
    source_sha256 = _sha256(path)
    if source_sha256 != CANONICAL_GRIP_SHA256:
        raise RuntimeError("canonical_grip_sha256_mismatch")
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        presets = _literal_assignment(tree, "PRESETS")
        source_topic = _literal_assignment(tree, "CMD_TOPIC")
    except (OSError, SyntaxError, ValueError) as exc:
        raise RuntimeError("canonical_grip_parse_failed") from exc
    expected = {
        "grab": {"method": 1, "pos": 800, "force": 20, "vel": 40},
        "release": {"method": 1, "pos": 2000, "force": 20, "vel": 60},
    }
    if presets != expected:
        raise RuntimeError("canonical_grip_presets_mismatch")
    if source_topic != COMMAND_TOPIC:
        raise RuntimeError("canonical_grip_topic_mismatch")
    return GripContract(
        source_path=path,
        source_sha256=source_sha256,
        grab={**expected["grab"], "cmd": 0},
        release={**expected["release"], "cmd": 0},
    )


def parse_request(packet: bytes) -> tuple[int, str]:
    if not packet or len(packet) > MAX_PACKET_BYTES:
        raise ProtocolError("request_size")
    try:
        text = packet.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ProtocolError("request_not_ascii") from exc
    fields = text.split("\t")
    if len(fields) != 3:
        raise ProtocolError("request_field_count")
    version, request_text, command = fields
    if version != PROTOCOL_VERSION:
        raise ProtocolError("protocol_version")
    if not request_text.isascii() or not request_text.isdecimal():
        raise ProtocolError("request_id")
    request_id = int(request_text)
    if not 1 <= request_id <= (1 << 63) - 1:
        raise ProtocolError("request_id")
    if command not in ALLOWED_COMMANDS:
        raise ProtocolError("command")
    return request_id, command


def format_response(
    request_id: int,
    *,
    ok: bool,
    command: str,
    state: str,
    first_publish_monotonic_ns: int = 0,
    published_before_ack: int = 0,
    planned_publish_count: int = 0,
    matched_subscribers: int = 0,
    detail: str = "none",
) -> bytes:
    if command not in ALLOWED_COMMANDS and command != "INVALID":
        raise ProtocolError("response_command")
    if state not in ALLOWED_STATES:
        raise ProtocolError("response_state")
    integers = (
        request_id,
        first_publish_monotonic_ns,
        published_before_ack,
        planned_publish_count,
        matched_subscribers,
    )
    if any(not isinstance(value, int) or value < 0 for value in integers):
        raise ProtocolError("response_integer")
    if not _DETAIL_RE.fullmatch(detail):
        raise ProtocolError("response_detail")
    return "\t".join(
        (
            PROTOCOL_VERSION,
            str(request_id),
            "OK" if ok else "ERR",
            command,
            state,
            str(first_publish_monotonic_ns),
            str(published_before_ack),
            str(planned_publish_count),
            str(matched_subscribers),
            "0",  # physical_confirmation: unavailable
            "0",  # state_observable: target firmware reports all zeros
            detail,
        )
    ).encode("ascii")


def _protobuf_varint(value: int) -> bytes:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError("protobuf_unsigned_integer")
    encoded = bytearray()
    while value > 0x7F:
        encoded.append((value & 0x7F) | 0x80)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def _protobuf_uint(field_number: int, value: int) -> bytes:
    if value == 0:
        return b""
    return _protobuf_varint(field_number << 3) + _protobuf_varint(value)


def _protobuf_message(field_number: int, payload: bytes) -> bytes:
    return (
        _protobuf_varint((field_number << 3) | 2)
        + _protobuf_varint(len(payload))
        + payload
    )


def serialize_hand_command_channel(
    sequence: int, config: dict[str, int], now_s: float
) -> bytes:
    """Encode the exact ``c89e5ab`` ``HandCommandChannel`` protobuf."""

    if (
        not isinstance(sequence, int)
        or isinstance(sequence, bool)
        or not 0 <= sequence <= 0xFFFFFFFF
    ):
        raise ValueError("hand_sequence_out_of_range")
    if not isinstance(now_s, (int, float)) or now_s < 0:
        raise ValueError("hand_timestamp_out_of_range")
    seconds = int(now_s)
    nanos = int((now_s % 1.0) * 1_000_000_000)
    milliseconds = int(now_s * 1_000)
    timestamp = (
        _protobuf_uint(1, seconds)
        + _protobuf_uint(2, nanos)
        + _protobuf_uint(3, milliseconds)
    )
    header = (
        _protobuf_uint(1, sequence)
        + _protobuf_message(2, timestamp)
        + _protobuf_uint(4, 1)  # ControlSource_MANUAL
    )

    claw_fields = {
        "cmd": 1,
        "pos": 2,
        "force": 3,
        "method": 4,
        "vel": 6,
    }
    claw = b""
    for name, field_number in claw_fields.items():
        value = config.get(name)
        if (
            not isinstance(value, int)
            or isinstance(value, bool)
            or not 0 <= value <= 0xFFFFFFFF
        ):
            raise ValueError(f"hand_{name}_out_of_range")
        claw += _protobuf_uint(field_number, value)

    left = _protobuf_message(3, claw)
    # Selecting the right agi_claw_cmd oneof with its default-valued message is
    # significant even though its nested payload is empty.
    right = _protobuf_message(3, b"")
    hand_command = _protobuf_message(1, left) + _protobuf_message(2, right)
    return _protobuf_message(1, header) + _protobuf_message(2, hand_command)


def _validate_private_paths(socket_path: Path, ready_file: Path) -> tuple[Path, Path]:
    if not socket_path.is_absolute() or not ready_file.is_absolute():
        raise SystemExit("socket 和 ready file 必须使用绝对路径")
    socket_parent = socket_path.parent.resolve(strict=True)
    ready_parent = ready_file.parent.resolve(strict=True)
    if socket_parent != ready_parent:
        raise SystemExit("socket 和 ready file 必须位于同一私有目录")
    if socket_path.name == ready_file.name:
        raise SystemExit("socket 和 ready file 必须是不同路径")
    metadata = socket_parent.stat()
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or stat.S_IMODE(metadata.st_mode) != 0o700
        or metadata.st_uid != os.geteuid()
    ):
        raise SystemExit("IPC 目录必须由当前用户拥有且 mode 精确为 0700")
    resolved_socket = socket_parent / socket_path.name
    resolved_ready = ready_parent / ready_file.name
    for target in (resolved_socket, resolved_ready):
        if os.path.lexists(target):
            raise SystemExit(f"拒绝覆盖已有 IPC 路径: {target}")
    return resolved_socket, resolved_ready


def _create_ready_marker(path: Path, payload: str) -> tuple[int, int]:
    temporary = path.parent / f".{path.name}.{os.getpid()}.tmp"
    descriptor = os.open(
        temporary,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
        0o600,
    )
    try:
        with os.fdopen(descriptor, "w", encoding="ascii") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path, follow_symlinks=False)
        marker = path.lstat()
        return marker.st_dev, marker.st_ino
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _unlink_owned_identity(path: Path, identity: tuple[int, int] | None) -> None:
    if identity is None:
        return
    try:
        metadata = path.lstat()
    except FileNotFoundError:
        return
    if (metadata.st_dev, metadata.st_ino) == identity and metadata.st_uid == os.geteuid():
        path.unlink()


def _load_ros() -> tuple[Any, Any, Any, Any, Any, Any]:
    import rclpy
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from ros2_plugin_proto.msg import RosMsgWrapper

    return (
        rclpy,
        QoSProfile,
        QoSHistoryPolicy,
        QoSReliabilityPolicy,
        QoSDurabilityPolicy,
        RosMsgWrapper,
    )


class GripPublisher:
    def __init__(self) -> None:
        (
            self.rclpy,
            qos_profile_type,
            qos_history,
            qos_reliability,
            qos_durability,
            self.wrapper_type,
        ) = _load_ros()
        self.rclpy.init()
        self.node = self.rclpy.create_node("a3p_serve_gripper_bridge")
        qos = qos_profile_type(
            history=qos_history.KEEP_LAST,
            depth=10,
            reliability=qos_reliability.BEST_EFFORT,
            durability=qos_durability.VOLATILE,
        )
        self.publisher = self.node.create_publisher(
            self.wrapper_type, COMMAND_TOPIC, qos
        )
        self.command_topic = COMMAND_TOPIC
        self.sequence = 0

    def close(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()

    def spin_once(self, timeout_sec: float) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def matched_subscribers(self) -> int:
        return int(self.publisher.get_subscription_count())

    def wait_for_subscriber(
        self,
        timeout_s: float,
        should_stop: Callable[[], bool] | None = None,
    ) -> int:
        deadline = time.monotonic() + timeout_s
        matched = self.matched_subscribers()
        while matched == 0 and time.monotonic() < deadline:
            if should_stop is not None and should_stop():
                raise RuntimeError("stop_requested_during_discovery")
            self.spin_once(0.05)
            matched = self.matched_subscribers()
        if matched == 0:
            raise RuntimeError("hand_command_no_subscriber")
        margin_deadline = time.monotonic() + 0.2
        while time.monotonic() < margin_deadline:
            if should_stop is not None and should_stop():
                raise RuntimeError("stop_requested_during_discovery")
            self.spin_once(0.02)
        matched = self.matched_subscribers()
        if matched == 0:
            raise RuntimeError("hand_command_no_subscriber")
        return matched

    def publish(self, config: dict[str, int]) -> int:
        now = time.time()
        raw = serialize_hand_command_channel(self.sequence, config, now)
        message = self.wrapper_type()
        message.serialization_type = "pb"
        message.context = ["aimdk.protocol.HandCommandChannel"]
        message.data = [bytes([value]) for value in raw]
        self.publisher.publish(message)
        self.sequence += 1
        return time.monotonic_ns()

    def publish_burst(
        self,
        config: dict[str, int],
        should_stop: Callable[[], bool] | None = None,
    ) -> tuple[int, int]:
        first_publish_ns = 0
        published = 0
        for index in range(BURST_COUNT):
            if should_stop is not None and should_stop():
                raise RuntimeError("stop_requested_during_burst")
            if self.matched_subscribers() <= 0:
                raise RuntimeError("hand_command_subscriber_lost_during_burst")
            published_ns = self.publish(config)
            published += 1
            if index == 0:
                first_publish_ns = published_ns
            if index + 1 < BURST_COUNT:
                deadline = time.monotonic() + BURST_PERIOD_S
                while time.monotonic() < deadline:
                    if should_stop is not None and should_stop():
                        raise RuntimeError("stop_requested_during_burst")
                    self.spin_once(0.01)
        return first_publish_ns, published


def _peer_is_current_user(client: socket.socket) -> bool:
    if not hasattr(socket, "SO_PEERCRED"):
        return False
    try:
        credentials = client.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12)
        _pid, uid, _gid = struct.unpack("3i", credentials)
    except (OSError, struct.error):
        return False
    return uid == os.geteuid()


def run_bridge(
    socket_path: Path,
    ready_file: Path,
    subscriber_timeout_s: float,
    contract: GripContract,
) -> int:
    if not hasattr(socket, "SO_PEERCRED"):
        raise RuntimeError("so_peercred_unavailable")
    socket_path, ready_file = _validate_private_paths(socket_path, ready_file)
    publisher = GripPublisher()
    server: socket.socket | None = None
    client: socket.socket | None = None
    socket_identity: tuple[int, int] | None = None
    marker_identity: tuple[int, int] | None = None
    stop_requested = False
    state = "IDLE"
    last_request_id = 0
    last_publish_ns = 0
    last_publish_count = 0
    last_planned_publish_count = 0

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGHUP, request_stop)

    try:
        # Topic discovery is confined to this optional bridge. The shell still
        # starts the body Runner if the hand subscriber is unavailable.
        matched = publisher.wait_for_subscriber(
            subscriber_timeout_s, lambda: stop_requested
        )
        server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        server.bind(str(socket_path))
        os.chmod(socket_path, 0o600)
        socket_metadata = socket_path.lstat()
        socket_identity = (socket_metadata.st_dev, socket_metadata.st_ino)
        server.listen(1)
        server.setblocking(False)
        marker_identity = _create_ready_marker(
            ready_file,
            (
                f"READY\t{PROTOCOL_VERSION}\tIDLE\t{matched}\t"
                f"grip_sha256={contract.source_sha256}\t"
                "transport=hand_command_topic_pb\t"
                f"topic={publisher.command_topic}\t"
                "physical_confirmation=0\tstate_observable=0\n"
            ),
        )
        print(
            "[a3p-gripper] READY/IDLE: DDS topic matched; no command sent; "
            f"topic={publisher.command_topic}; "
            f"matched_subscribers={matched}; "
            "physical confirmation unavailable",
            flush=True,
        )

        while not stop_requested:
            publisher.spin_once(0.0)
            readers = [server]
            if client is not None:
                readers.append(client)
            readable, _, _ = select.select(readers, [], [], 0.02)
            if server in readable:
                accepted, _ = server.accept()
                accepted.setblocking(True)
                if client is not None or not _peer_is_current_user(accepted):
                    accepted.close()
                else:
                    client = accepted
            if client is None or client not in readable:
                continue
            packet, _ancillary, flags, _address = client.recvmsg(MAX_PACKET_BYTES)
            if not packet:
                client.close()
                client = None
                continue
            if flags & getattr(socket, "MSG_TRUNC", 0):
                packet = b""

            request_id = 0
            command = "INVALID"
            response = b""
            try:
                request_id, command = parse_request(packet)
                if request_id <= last_request_id:
                    raise ProtocolError("request_not_monotonic")
                last_request_id = request_id
                matched = publisher.matched_subscribers()

                if command == "STATUS":
                    response = format_response(
                        request_id,
                        ok=state != "FAULT",
                        command=command,
                        state=state,
                        first_publish_monotonic_ns=last_publish_ns,
                        published_before_ack=last_publish_count,
                        planned_publish_count=last_planned_publish_count,
                        matched_subscribers=matched,
                        detail=(
                            "none" if state != "FAULT" else "transport_not_ready"
                        ),
                    )
                elif command == "OPEN":
                    # The vendor hand state is not observable here.  Never
                    # infer a physical gate from the last published command:
                    # doing that suppresses valid retries after packet loss.
                    try:
                        last_publish_ns, last_publish_count = publisher.publish_burst(
                            contract.release, lambda: stop_requested
                        )
                        last_planned_publish_count = BURST_COUNT
                    except Exception as exc:
                        state = "FAULT"
                        detail = str(exc)
                        if not _DETAIL_RE.fullmatch(detail):
                            detail = "open_publish_failed"
                        raise ProtocolError(detail) from exc
                    state = "OPEN"
                    receipt_ready = publisher.matched_subscribers()
                    print(
                        "[a3p-gripper] OPEN published on c89 topic: "
                        f"count={last_publish_count} "
                        f"matched_subscribers={publisher.matched_subscribers()} "
                        f"first_dispatch_monotonic_ns={last_publish_ns}; "
                        "physical confirmation unavailable",
                        flush=True,
                    )
                    response = format_response(
                        request_id,
                        ok=True,
                        command=command,
                        state=state,
                        first_publish_monotonic_ns=last_publish_ns,
                        published_before_ack=last_publish_count,
                        planned_publish_count=BURST_COUNT,
                        matched_subscribers=receipt_ready,
                    )
                elif command == "GRAB":
                    try:
                        last_publish_ns, last_publish_count = publisher.publish_burst(
                            contract.grab, lambda: stop_requested
                        )
                        last_planned_publish_count = BURST_COUNT
                    except Exception as exc:
                        state = "FAULT"
                        detail = str(exc)
                        if not _DETAIL_RE.fullmatch(detail):
                            detail = "grab_publish_failed"
                        raise ProtocolError(detail) from exc
                    state = "GRABBED"
                    receipt_ready = publisher.matched_subscribers()
                    print(
                        "[a3p-gripper] GRAB published on c89 topic: "
                        f"count={last_publish_count} "
                        f"matched_subscribers={publisher.matched_subscribers()} "
                        f"first_dispatch_monotonic_ns={last_publish_ns}; "
                        "physical confirmation unavailable",
                        flush=True,
                    )
                    response = format_response(
                        request_id,
                        ok=True,
                        command=command,
                        state=state,
                        first_publish_monotonic_ns=last_publish_ns,
                        published_before_ack=last_publish_count,
                        planned_publish_count=BURST_COUNT,
                        matched_subscribers=receipt_ready,
                    )
                elif command == "RELEASE":
                    try:
                        last_publish_ns, last_publish_count = publisher.publish_burst(
                            contract.release, lambda: stop_requested
                        )
                        last_planned_publish_count = BURST_COUNT
                    except Exception as exc:
                        state = "FAULT"
                        detail = str(exc)
                        if not _DETAIL_RE.fullmatch(detail):
                            detail = "release_publish_failed"
                        raise ProtocolError(detail) from exc
                    state = "RELEASED"
                    matched = publisher.matched_subscribers()
                    response = format_response(
                        request_id,
                        ok=True,
                        command=command,
                        state=state,
                        first_publish_monotonic_ns=last_publish_ns,
                        published_before_ack=last_publish_count,
                        planned_publish_count=BURST_COUNT,
                        matched_subscribers=matched,
                    )
                    print(
                        "[a3p-gripper] RELEASE published on c89 topic; "
                        f"first_dispatch_monotonic_ns={last_publish_ns}; "
                        "physical confirmation unavailable",
                        flush=True,
                    )
                else:  # SHUTDOWN
                    state = "STOPPING"
                    response = format_response(
                        request_id,
                        ok=True,
                        command=command,
                        state=state,
                        matched_subscribers=matched,
                    )
                    stop_requested = True
            except ProtocolError as exc:
                response = format_response(
                    request_id,
                    ok=False,
                    command=command,
                    state=state,
                    first_publish_monotonic_ns=last_publish_ns,
                    published_before_ack=last_publish_count,
                    planned_publish_count=last_planned_publish_count,
                    matched_subscribers=publisher.matched_subscribers(),
                    detail=str(exc),
                )
            if response and client is not None:
                try:
                    client.sendall(response)
                except OSError:
                    client.close()
                    client = None
    finally:
        if client is not None:
            client.close()
        if server is not None:
            server.close()
        _unlink_owned_identity(ready_file, marker_identity)
        _unlink_owned_identity(socket_path, socket_identity)
        publisher.close()
    return 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "A3P 发球常驻夹爪 bridge；启动仅进入 IDLE，"
            "接受彼此独立且幂等的 OPEN / GRAB / RELEASE 指令"
        )
    )
    parser.add_argument("--socket", type=Path)
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--grip-source", type=Path)
    parser.add_argument("--subscriber-timeout-s", type=float, default=5.0)
    parser.add_argument("--allow-publish", action="store_true")
    parser.add_argument("--confirm-real-gripper", action="store_true")
    parser.add_argument(
        "--check-contract",
        action="store_true",
        help="只检查 canonical grip.py 和 ROS imports；不建节点、不发布",
    )
    args = parser.parse_args(argv)
    if args.grip_source is None:
        parser.error("必须给 --grip-source 的绝对路径")
    if args.check_contract:
        if any(
            value is not None for value in (args.socket, args.ready_file)
        ) or args.allow_publish or args.confirm_real_gripper:
            parser.error("--check-contract 不能与 IPC 或真实发布 flags 组合")
        return args
    if args.socket is None or args.ready_file is None:
        parser.error("运行 bridge 必须同时给 --socket 与 --ready-file")
    if args.allow_publish != args.confirm_real_gripper or not args.allow_publish:
        parser.error(
            "运行 bridge 必须同时给 --allow-publish 与 --confirm-real-gripper"
        )
    if not 0.1 <= args.subscriber_timeout_s <= 30.0:
        parser.error("--subscriber-timeout-s 必须在 0.1..30 秒")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        contract = load_grip_contract(args.grip_source)
        if args.check_contract:
            _load_ros()
            print(
                "[a3p-gripper] contract/import check PASS; "
                "no ROS node or command",
                flush=True,
            )
            return 0
        return run_bridge(
            args.socket,
            args.ready_file,
            args.subscriber_timeout_s,
            contract,
        )
    except (OSError, RuntimeError) as exc:
        print(f"[a3p-gripper] ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
