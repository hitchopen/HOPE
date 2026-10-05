#!/usr/bin/env python3
"""Read a Linux Xbox controller and optionally publish Foxglove input preview.

Default: JSON on stdout. --ros: std_msgs/String /hope/runner/xbox_preview.
--ros --control sends LT-enabled velocity and A/B/X/Y fixed mode service
requests to the single Runner. It never publishes joint/body commands.
Requires evdev; --ros additionally needs the existing ROS 2 environment.
"""

from __future__ import annotations

import argparse
import json
import math
import time
import uuid
import os
import subprocess
from pathlib import Path
import sys

from hope_xbox_actions import ActionSequence, ButtonEdges, EmergencyChord, face_button_names


class StreamCadence:
    """Send semantic edges immediately; idle heartbeats at 10 Hz.

    Active control still publishes every 50 Hz input tick. No stored message
    is replayed: the caller always constructs a new timestamp and sequence.
    """
    def __init__(self):
        self.last = -math.inf
        self.key = None

    def ready(self, now, key, *, active=False):
        if active or key != self.key or now - self.last >= 0.1 - 1e-9:
            self.last, self.key = now, key
            return True
        return False


class ConnectionRecovery:
    """Recreate DDS after a replaced network interface or lost monitor stream.

    A stale safety stream never authorizes input. Exiting with a failure lets
    the dedicated systemd input service rebuild its route-derived DDS profile.
    """
    def __init__(self, now):
        self.started = now
        self.route = None

    def reason(self, now, route, safety_received):
        if route is not None:
            if self.route is not None and route != self.route:
                return "Robot network changed; reconnecting Xbox transport"
            self.route = route
        if now - max(self.started, safety_received) > 8.0:
            return "Safety status stream lost; reconnecting Xbox transport"
        return None


def robot_route(peer):
    try:
        result = subprocess.run(["ip", "-j", "route", "get", peer],
                                capture_output=True, text=True, timeout=.2, check=True)
        route = json.loads(result.stdout)[0]
        interface = route["dev"]
        index = Path("/sys/class/net", interface, "ifindex").read_text().strip()
        return interface, index, route.get("prefsrc", route.get("src", ""))
    except (OSError, subprocess.SubprocessError, ValueError, KeyError, IndexError):
        return None


def normalize_axis(value: int, minimum: int, maximum: int, deadzone: float) -> float:
    if maximum <= minimum or not 0 <= deadzone < 1:
        raise ValueError("invalid axis range/deadzone")
    midpoint = (minimum + maximum) / 2
    unit = max(-1.0, min(1.0, (value - midpoint) / ((maximum - minimum) / 2)))
    return (
        0.0
        if abs(unit) <= deadzone
        else math.copysign((abs(unit) - deadzone) / (1 - deadzone), unit)
    )


class EnableLatch:
    """Reconnect requires release, centered sticks, then a new LT press."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.released = False
        self.previous_pressed = False
        self.enabled = False

    def sample(self, connected: bool, pressed: bool, axes: list[float]) -> tuple[bool, str]:
        if (
            not connected
            or len(axes) != 3
            or not all(math.isfinite(x) and abs(x) <= 1 for x in axes)
        ):
            self.reset()
            return False, "DISCONNECTED"
        neutral = all(abs(x) < 1e-6 for x in axes)
        if not pressed:
            self.enabled = False
            self.released = neutral
        elif not self.previous_pressed:
            self.enabled = self.released and neutral
            self.released = False
        self.previous_pressed = pressed
        return self.enabled, "ENABLED" if self.enabled else "CENTER_AND_PRESS_LT"


class TriggerLatch:
    """LT is ABS_Z [0,1023] on the deployed Xbox; use range-normalized hysteresis."""
    def __init__(self):
        self.pressed = False

    def sample(self, value, minimum, maximum):
        if maximum <= minimum:
            raise ValueError("invalid LT range")
        unit = max(0., min(1., (value - minimum) / (maximum - minimum)))
        self.pressed = unit >= (.35 if self.pressed else .55)
        return self.pressed, unit


def run(args):
    from evdev import InputDevice, ecodes, list_devices

    node = publisher = control_publisher = ros = None
    latch = EnableLatch()
    runner = [0, 0.0]
    runner_state = [None, 0.0]
    trigger = TriggerLatch()
    buttons = ButtonEdges()
    actions = ActionSequence()
    clients = {}
    emergency = EmergencyChord()
    estop_received = [None, 0.]
    estop_client = estop_future = None
    estop_started = 0.
    future = None
    future_started = 0.
    if args.ros:
        import rclpy as ros
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, Float64MultiArray, String

        ros.init(args=[])
        node = ros.create_node("hope_xbox_input_preview")
        publisher = node.create_publisher(String, "/hope/runner/xbox_preview", 1)
        if args.control:
            from std_srvs.srv import Trigger
            sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
            from hope_runner_control_core import decode_runner_state
            from hope_observer_core import DecodeError
            clients = {name: node.create_client(Trigger, "/hope/runner/" + name)
                       for name in ActionSequence.SERVICES.values()}
            estop_client = node.create_client(Trigger, "/hope/safety/trigger_estop")
            def on_estop(message):
                emergency.observe_latch(message.data)
                estop_received[:] = [message.data, time.monotonic()]
            node.create_subscription(Bool, "/hope/safety/estop_latched", on_estop, 10)
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            control_publisher = node.create_publisher(
                Float64MultiArray, "/hope/runner/teleop_input_hdu_flat", qos
            )

            def on_runner(message):
                values = list(message.data)
                if (
                    len(values) != 10
                    or not all(math.isfinite(x) for x in values)
                    or values[0] != 1
                    or values[2] != 1
                    or not 1 <= values[1] < 2**52
                    or int(values[1]) != values[1]
                ):
                    return
                if runner[0] != int(values[1]):
                    latch.reset()
                    buttons.reset()
                runner[:] = [int(values[1]), time.monotonic()]

            node.create_subscription(
                Float64MultiArray, "/hope/runner/teleop_state_hdu_flat", on_runner, qos
            )
            def on_state(message):
                try:
                    decoded = decode_runner_state(message.data)
                except DecodeError:
                    return
                if runner_state[0] is None or runner_state[0].boot_id != decoded.boot_id:
                    buttons.reset()
                    actions.cancel("Runner connected; release A/B/X/Y before pressing a mode key")
                runner_state[:] = [decoded, time.monotonic()]

            node.create_subscription(Float64MultiArray, "/hope/runner/state_hdu_flat", on_state, qos)
    device = None
    session = uuid.uuid4().hex
    started = time.monotonic()
    recovery = ConnectionRecovery(started)
    next_health_check = started
    next_discovery = 0.0
    sequence = 0
    preview_cadence, control_cadence = StreamCadence(), StreamCadence()
    try:
        while args.seconds <= 0 or time.monotonic() - started < args.seconds:
            loop = time.monotonic()
            if node is not None:
                # Drain more than one ready callback: both Runner streams can
                # arrive at 50 Hz alongside the safety heartbeat and RPCs.
                for _ in range(8):
                    ros.spin_once(node, timeout_sec=0)
            if args.control and loop >= next_health_check:
                next_health_check = loop + .5
                reason = recovery.reason(loop, robot_route(os.environ.get("HOPE_XBOX_HDU_IP", "10.42.20.10")), estop_received[1])
                if reason:
                    print(reason, file=sys.stderr, flush=True)
                    raise SystemExit(75)
            error = ""
            axes = [0.0, 0.0, 0.0]
            lt, lt_value = False, 0.
            face_keys = set()
            lb = rb = False
            if device is None and loop >= next_discovery:
                next_discovery = loop + .2
                matches = []
                for path in list_devices():
                    candidate = None
                    try:
                        candidate = InputDevice(path)
                        capabilities = candidate.capabilities()
                    except OSError:
                        if candidate is not None:
                            candidate.close()
                        continue
                    if (
                        candidate.info.vendor == 0x045E
                        and candidate.info.product == 0x0B12
                        and all(
                            code
                            in [
                                item if isinstance(item, int) else item[0]
                                for item in capabilities.get(ecodes.EV_ABS, [])
                            ]
                            for code in (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_Z)
                        )
                    ):
                        matches.append(candidate)
                    else:
                        candidate.close()
                if len(matches) == 1:
                    device = matches[0]
                    session = uuid.uuid4().hex
                    sequence = 0
                    latch.reset()
                    buttons.reset()
                    trigger = TriggerLatch()
                else:
                    for candidate in matches:
                        candidate.close()
                    error = "MULTIPLE_XBOX_DEVICES" if matches else "NO_XBOX_DEVICE"
            if device is not None:
                try:
                    # Query current kernel state: do not replay an event backlog
                    # after scheduling stalls; ioctl fails on device removal.
                    for i, code in enumerate(
                        (ecodes.ABS_Y, ecodes.ABS_X, ecodes.ABS_RX)
                    ):
                        info = device.absinfo(code)
                        axes[i] = -normalize_axis(
                            info.value, info.min, info.max, args.deadzone
                        )
                    info = device.absinfo(ecodes.ABS_Z)
                    lt, lt_value = trigger.sample(info.value, info.min, info.max)
                    keys = device.active_keys()
                    lb, rb = ecodes.BTN_TL in keys, ecodes.BTN_TR in keys
                    face_keys = face_button_names(keys)
                except OSError as exc:
                    error = str(exc)
                    device.close()
                    device = None
                    next_discovery = 0.0
                    latch.reset()
                    buttons.reset()
                    actions.cancel("Controller disconnected; reconnecting automatically")
            current_state = runner_state[0] if loop - runner_state[1] <= 1.0 else None
            emergency.sample(lb, rb)
            estop_known_clear = estop_received[0] is False and loop - estop_received[1] <= 1.5
            if emergency.latched:
                actions.cancel(emergency.status)
                latch.reset()
                buttons.reset()
            elif args.control and not estop_known_clear:
                actions.cancel("Waiting for fresh E-stop status")
            elif actions.status == "Waiting for fresh E-stop status":
                actions.status = "A prepare · B serve · X receive Ready · Y Teleop from Stand only"
            # E-stop has its own async client and never waits behind a mode RPC.
            if args.control and estop_future is not None:
                if estop_future.done():
                    try:
                        receipt = estop_future.result()
                        if not receipt.success:
                            emergency.status = "E-STOP incomplete: " + receipt.message
                    except Exception as exc:
                        emergency.status = "E-STOP service failed: " + str(exc)
                    estop_future = None
                elif loop - estop_started > 4.:
                    estop_future.cancel()
                    estop_future = None
                    emergency.status = "E-STOP response timeout; check hardware stop"
            if args.control and emergency.pending and estop_future is None:
                if estop_client.service_is_ready():
                    estop_future = estop_client.call_async(Trigger.Request())
                    estop_started = loop
                    emergency.pending = False
                else:
                    emergency.status = "E-STOP requested; waiting for E-stop service"
            if future is not None:
                if not actions.busy:
                    future.cancel()
                    future = None
                elif future.done():
                    try:
                        response = future.result()
                        actions.acknowledge(response.success, response.message)
                    except Exception as exc:
                        actions.acknowledge(False, str(exc))
                    future = None
                elif loop - future_started > 4.:
                    future.cancel()
                    future = None
                    actions.cancel("Runner service timeout; press again after checking status")
            edge = buttons.sample(device is not None and not emergency.latched and
                                  (not args.control or (current_state is not None and estop_known_clear)), face_keys)
            if args.control and not emergency.latched and estop_known_clear:
                if edge and actions.request(edge, current_state, loop):
                    latch.reset()
                service = actions.advance(current_state, loop, device is not None)
                if service is not None:
                    client = clients[service]
                    if not client.service_is_ready():
                        actions.cancel(f"Runner service unavailable: {service}; press again")
                    else:
                        future = client.call_async(Trigger.Request())
                        future_started = loop
            enabled, state = latch.sample(device is not None, lt, axes)
            if actions.busy or emergency.latched or (args.control and not estop_known_clear):
                latch.reset()
                enabled = False
                state = ("E_STOP" if emergency.latched else
                         "WAITING_FOR_SAFETY_STATUS" if args.control and not estop_known_clear else
                         "MODE_CHANGE_IN_PROGRESS")
            velocity = (
                [
                    axes[0] * (args.forward if axes[0] >= 0 else args.backward),
                    axes[1] * args.lateral,
                    axes[2] * args.yaw,
                ]
                if enabled
                else [0.0, 0.0, 0.0]
            )
            sequence += 1
            packet = {
                "schema": 1,
                "purpose": "XBOX_INPUT",
                "session": session,
                "sequence": sequence,
                "source_wall_ms": int(time.time() * 1000),
                "connected": device is not None,
                "device": device.name if device else "",
                "lt": lt,
                "lt_value": lt_value,
                "buttons": sorted(face_keys),
                "action_status": (emergency.status if emergency.latched else
                                  "Waiting for fresh E-stop status" if args.control and not estop_known_clear else
                                  "No fresh Runner; start the system before using mode keys" if args.control and current_state is None else
                                  actions.status),
                "lb": lb,
                "rb": rb,
                "estop_requested": emergency.latched,
                "action_pending": actions.busy,
                "enabled": enabled,
                "axes": axes,
                "velocity": velocity,
                "state": state,
                "error": error,
                "control_stream": args.control,
            }
            payload = json.dumps(packet, separators=(",", ":"))
            if publisher is not None:
                preview_key = (packet["connected"], packet["enabled"], state,
                               packet["action_status"], tuple(packet["buttons"]),
                               packet["estop_requested"], packet["action_pending"])
                if preview_cadence.ready(loop, preview_key):
                    message = String()
                    message.data = payload
                    publisher.publish(message)
                if control_publisher is not None:
                    if runner[0] and time.monotonic() - runner[1] <= 1.0:
                        mode = current_state.run_mode if current_state is not None else None
                        neutral = all(abs(x) < 1e-6 for x in axes)
                        control_key = (runner[0], device is not None, enabled, neutral,
                                       mode, actions.busy, emergency.latched)
                        # Keep the complete 50 Hz stream in TELEOP, while LT is
                        # enabled, or during a mode transition. Disable, device
                        # removal, neutral/rearm and boot edges bypass the gate.
                        if control_cadence.ready(loop, control_key, active=(
                                enabled or mode == "TELEOP" or actions.busy)):
                            message = Float64MultiArray()
                            source_id = int(session[:13], 16) or 1
                            message.data = [
                                1.0,
                                float(runner[0]),
                                float(source_id),
                                float(sequence),
                                packet["source_wall_ms"] * 0.001,
                                float(device is not None),
                                float(enabled),
                                *velocity,
                                float(neutral),
                            ]
                            control_publisher.publish(message)
                    else:
                        latch.reset()
            elif sequence % 10 == 1:
                print(payload, flush=True)
            time.sleep(max(0, 0.02 - (time.monotonic() - loop)))
    except Exception:
        # ROS handles SIGTERM by shutting down its context, which can interrupt
        # spin/publish between two loop statements during a service restart.
        if node is None or ros.ok():
            raise
    finally:
        if device is not None:
            device.close()
        if node is not None:
            node.destroy_node()
            if ros.ok():
                ros.shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ros", action="store_true")
    p.add_argument(
        "--control",
        action="store_true",
        help="send LT velocity and A/B/X/Y mode actions through existing Runner services; requires --ros",
    )
    p.add_argument("--seconds", type=float, default=0)
    p.add_argument("--deadzone", type=float, default=0.12)
    p.add_argument("--forward", type=float, default=0.3)
    p.add_argument("--backward", type=float, default=0.2)
    p.add_argument("--lateral", type=float, default=0.15)
    p.add_argument("--yaw", type=float, default=0.4)
    args = p.parse_args()
    if args.control and not args.ros:
        p.error("--control requires --ros")
    if (
        not all(
            math.isfinite(x) and x >= 0
            for x in (args.seconds, args.forward, args.backward, args.lateral, args.yaw)
        )
        or not math.isfinite(args.deadzone)
        or not 0 <= args.deadzone < 1
    ):
        p.error("limits must be finite/nonnegative and deadzone in [0,1)")
    if args.forward > 0.8 or args.backward > 0.5 or args.lateral > 0.3 or args.yaw > 1:
        p.error(
            "limits exceed Runner contract: forward .8, backward .5, lateral .3 m/s; yaw 1 rad/s"
        )
    run(args)
