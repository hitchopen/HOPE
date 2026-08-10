#!/usr/bin/env python3
"""HOPE A3 monitoring and narrowly scoped safety publisher for Foxglove.

Reads `chronyc tracking`, local systemd state, one timestamped IMU topic, the
vendor TF tree, and vendor joint states. The one write path is an explicit
Trigger service that can assert (but never clear) the A3 vendor software
E-stop. Motive/NatNet and mocap diagnostics remain on the external laptop.
Publishes:

  /hope/ntp/offset_ms           std_msgs/Float64  chrony System time offset
  /hope/ntp/skew_ppm            std_msgs/Float64
  /hope/ntp/root_dispersion_ms  std_msgs/Float64
  /hope/ntp/utc_qualified       std_msgs/Bool     Leap Normal + selected source
  /hope/ntp/gate_pass           std_msgs/Bool     qualified + offset/skew gates
  /hope/ntp/text                std_msgs/String   human-readable offset in ms
  /hope/clock/message_latency_ms std_msgs/Float64 A3 ROS time - message stamp
  /hope/clock/message_fresh      std_msgs/Bool
  /hope/system/cpu_load_percent  std_msgs/Float64 aggregate CPU busy percentage
  /hope/vendor/agibot_pm_active  std_msgs/Bool     local HDU systemd unit state
  /hope/v17/system/hdu_active    std_msgs/Bool     V17 HDU observer unit state
  /hope/vendor/tf_ready          std_msgs/Bool     fresh reference->pelvis TF
  /hope/robot_description        std_msgs/String   URDF only while TF is fresh
  /hope/safety/estop_ready       std_msgs/Bool     live vendor E-stop RPC matched
  /hope/pelvis/pose             geometry_msgs/PoseStamped  reference->pelvis TF
  /hope/pelvis/text             std_msgs/String   human-readable pose (or TF error)
  /hope/joints/fresh            std_msgs/Bool     all configured groups are fresh
  /hope/joints/text             std_msgs/String   group freshness details
  /joint_states                 sensor_msgs/JointState  bounded-rate merged joints

Service:
  /hope/safety/trigger_estop    std_srvs/Trigger  assert vendor software E-stop
"""

from concurrent.futures import Future, ThreadPoolExecutor
import math
from pathlib import Path
import threading
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    from ros2_plugin_proto.srv import RosRpcWrapper
except ImportError:  # Status still works; the E-stop proxy is not advertised.
    RosRpcWrapper = None

from hope_monitor_core import (
    NtpProbeResult,
    ServiceProbeResult,
    build_software_estop_request,
    cpu_load_percent,
    decode_software_estop_response,
    load_robot_urdf_for_foxglove,
    message_latency_ms,
    parse_proc_stat_cpu,
    probe_ntp,
    probe_systemd_service,
    stale_sources,
    timestamp_age_s,
)

VENDOR_JOINT_TOPICS = [
    "/motion/control/leg_joint_state",
    "/motion/control/arm_joint_state",
    "/motion/control/hand_joint_state",
    "/motion/control/neck_joint_state",
    "/motion/control/waist_joint_state",
]

class HopeMonitor(Node):
    def __init__(self):
        super().__init__("hope_monitor")
        self.declare_parameter("period_s", 1.0)
        self.declare_parameter("pelvis_frame", "pelvis_link")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("ntp_max_offset_ms", 10.0)
        self.declare_parameter("ntp_max_skew_ppm", 5.0)
        self.declare_parameter("joint_publish_hz", 20.0)
        self.declare_parameter("joint_stale_after_s", 0.5)
        self.declare_parameter(
            "message_latency_topic", "/ros2/body_drive/pelvis_imu/data"
        )
        self.declare_parameter("message_latency_publish_hz", 20.0)
        self.declare_parameter("message_latency_stale_after_s", 0.5)
        self.declare_parameter("agibot_pm_unit", "agibot_pm.service")
        self.declare_parameter("hdu_runtime_unit", "hope-v17-observer.service")
        self.declare_parameter("cpu_publish_period_s", 1.0)
        self.declare_parameter("tf_stale_after_s", 0.5)
        self.declare_parameter("robot_description_publish_period_s", 5.0)
        self.declare_parameter("robot_model_info_path", "/agibot/data/info/model")
        self.declare_parameter(
            "robot_model_root", "/opt/agibot/share/robot_model/models"
        )
        self.declare_parameter(
            "robot_asset_root_url", "http://localhost:8000/urdf"
        )

        self.pub_offset = self.create_publisher(Float64, "/hope/ntp/offset_ms", 10)
        self.pub_skew = self.create_publisher(Float64, "/hope/ntp/skew_ppm", 10)
        self.pub_disp = self.create_publisher(
            Float64, "/hope/ntp/root_dispersion_ms", 10
        )
        self.pub_qualified = self.create_publisher(Bool, "/hope/ntp/utc_qualified", 10)
        self.pub_ntp_gate = self.create_publisher(Bool, "/hope/ntp/gate_pass", 10)
        self.pub_ntp_text = self.create_publisher(String, "/hope/ntp/text", 10)
        self.pub_message_latency = self.create_publisher(
            Float64, "/hope/clock/message_latency_ms", 10
        )
        self.pub_message_fresh = self.create_publisher(
            Bool, "/hope/clock/message_fresh", 10
        )
        self.pub_message_text = self.create_publisher(
            String, "/hope/clock/message_text", 10
        )
        self.pub_cpu_load = self.create_publisher(
            Float64, "/hope/system/cpu_load_percent", 10
        )
        self.pub_cpu_text = self.create_publisher(
            String, "/hope/system/cpu_text", 10
        )
        self.pub_pm_active = self.create_publisher(
            Bool, "/hope/vendor/agibot_pm_active", 10
        )
        self.pub_pm_text = self.create_publisher(
            String, "/hope/vendor/agibot_pm_text", 10
        )
        self.pub_hdu_active = self.create_publisher(
            Bool, "/hope/v17/system/hdu_active", 10
        )
        self.pub_hdu_text = self.create_publisher(
            String, "/hope/v17/system/hdu_text", 10
        )
        self.pub_tf_ready = self.create_publisher(
            Bool, "/hope/vendor/tf_ready", 10
        )
        robot_description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_robot_description = self.create_publisher(
            String, "/hope/robot_description", robot_description_qos
        )
        self.pub_estop_ready = self.create_publisher(
            Bool, "/hope/safety/estop_ready", 10
        )
        self.pub_estop_text = self.create_publisher(
            String, "/hope/safety/estop_text", 10
        )
        self.pub_joints = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_joints_fresh = self.create_publisher(Bool, "/hope/joints/fresh", 10)
        self.pub_joints_text = self.create_publisher(String, "/hope/joints/text", 10)
        self.pub_pelvis = self.create_publisher(PoseStamped, "/hope/pelvis/pose", 10)
        self.pub_pelvis_text = self.create_publisher(String, "/hope/pelvis/text", 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._robot_urdf_xml = None
        self._robot_description_visible = None
        self._last_robot_description_publish_monotonic = float("-inf")
        try:
            model, urdf_path, self._robot_urdf_xml = load_robot_urdf_for_foxglove(
                model_info_path=str(
                    self.get_parameter("robot_model_info_path").value
                ),
                model_root=str(self.get_parameter("robot_model_root").value),
                public_asset_root_url=str(
                    self.get_parameter("robot_asset_root_url").value
                ),
            )
            self.get_logger().info(
                f"loaded {model} URDF for TF-gated Foxglove display: {urdf_path}"
            )
        except (OSError, ValueError) as exc:
            self.get_logger().error(
                f"A3 URDF unavailable; table-only display will remain active: {exc}"
            )

        latency_topic = str(self.get_parameter("message_latency_topic").value)
        self._message_latency_topic = latency_topic
        self._latest_message_latency_ms = None
        self._message_received_monotonic = None
        self._previous_cpu_times = None
        self.create_subscription(
            Imu,
            latency_topic,
            self._on_latency_message,
            qos_profile_sensor_data,
        )

        self._vendor_callback_group = ReentrantCallbackGroup()
        self._vendor_estop_client = None
        self._estop_proxy_service = None
        self._estop_service_lock = threading.Lock()
        self._estop_call_in_progress = False
        self._last_estop_backend_ready = None
        if RosRpcWrapper is not None:
            self._vendor_estop_client = self.create_client(
                RosRpcWrapper,
                "/aimdk_2Eprotocol_2EHalEmergencyService/SetEmergencyCommand",
                callback_group=self._vendor_callback_group,
            )

        self._joint_groups = {}
        self._joint_received_monotonic = {}
        for topic in VENDOR_JOINT_TOPICS:
            self.create_subscription(
                JointState,
                topic,
                lambda msg, source=topic: self._on_joint_state(source, msg),
                10,
            )

        period = float(self.get_parameter("period_s").value)
        joint_publish_hz = float(self.get_parameter("joint_publish_hz").value)
        latency_publish_hz = float(
            self.get_parameter("message_latency_publish_hz").value
        )
        cpu_publish_period_s = float(
            self.get_parameter("cpu_publish_period_s").value
        )
        if period <= 0.0:
            raise ValueError("period_s must be positive")
        if joint_publish_hz <= 0.0:
            raise ValueError("joint_publish_hz must be positive")
        if latency_publish_hz <= 0.0:
            raise ValueError("message_latency_publish_hz must be positive")
        if cpu_publish_period_s <= 0.0:
            raise ValueError("cpu_publish_period_s must be positive")
        if float(self.get_parameter("tf_stale_after_s").value) <= 0.0:
            raise ValueError("tf_stale_after_s must be positive")
        if (
            float(self.get_parameter("robot_description_publish_period_s").value)
            <= 0.0
        ):
            raise ValueError("robot_description_publish_period_s must be positive")

        self._probe_pool = ThreadPoolExecutor(
            max_workers=3, thread_name_prefix="hope-monitor-probe"
        )
        self._ntp_future: Future[NtpProbeResult] | None = None
        self._pm_future: Future[ServiceProbeResult] | None = None
        self._hdu_future: Future[ServiceProbeResult] | None = None
        self.create_timer(period, self._poll_ntp)
        self.create_timer(period, self._poll_pm)
        self.create_timer(period, self._poll_hdu)
        self.create_timer(cpu_publish_period_s, self._poll_cpu)
        self.create_timer(0.5, self._poll_estop_backend)
        self.create_timer(0.2, self._poll_pelvis)
        self.create_timer(1.0 / joint_publish_hz, self._publish_joint_states)
        self.create_timer(1.0 / latency_publish_hz, self._publish_message_latency)

    # ---- CPU load ----------------------------------------------------------
    def _poll_cpu(self):
        try:
            current = parse_proc_stat_cpu(
                Path("/proc/stat").read_text(encoding="utf-8")
            )
            previous = self._previous_cpu_times
            self._previous_cpu_times = current
            if previous is None:
                self.pub_cpu_text.publish(String(data="CPU LOAD WARMING UP"))
                return
            load = cpu_load_percent(previous, current)
        except (OSError, ValueError) as exc:
            self.pub_cpu_text.publish(
                String(data=f"CPU LOAD UNAVAILABLE | {exc}")
            )
            self.get_logger().warn(
                f"CPU load probe unavailable: {exc}", throttle_duration_sec=30
            )
            return

        self.pub_cpu_load.publish(Float64(data=load))
        self.pub_cpu_text.publish(
            String(data=f"A3 aggregate CPU load = {load:.1f}%")
        )

    # ---- timestamp latency -------------------------------------------------
    def _on_latency_message(self, msg: Imu):
        try:
            latency_ms = message_latency_ms(
                self.get_clock().now().nanoseconds,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
            )
        except ValueError as exc:
            self._latest_message_latency_ms = None
            self._message_received_monotonic = time.monotonic()
            self.pub_message_text.publish(String(data=str(exc)))
            return
        self._latest_message_latency_ms = latency_ms
        self._message_received_monotonic = time.monotonic()

    def _publish_message_latency(self):
        stale_after_s = float(
            self.get_parameter("message_latency_stale_after_s").value
        )
        if stale_after_s <= 0.0:
            self.get_logger().error("message_latency_stale_after_s must be positive")
            return
        received = self._message_received_monotonic
        fresh = (
            received is not None
            and time.monotonic() - received <= stale_after_s
            and self._latest_message_latency_ms is not None
            and math.isfinite(self._latest_message_latency_ms)
        )
        self.pub_message_fresh.publish(Bool(data=fresh))
        if not fresh:
            self.pub_message_text.publish(
                String(data=f"NO FRESH TIMESTAMP | {self._message_latency_topic}")
            )
            return
        value = float(self._latest_message_latency_ms)
        self.pub_message_latency.publish(Float64(data=value))
        self.pub_message_text.publish(
            String(
                data=(
                    f"A3 ROS clock - {self._message_latency_topic} header stamp "
                    f"= {value:+.3f} ms"
                )
            )
        )

    # ---- joint aggregation -------------------------------------------------
    def _on_joint_state(self, source: str, msg: JointState):
        self._joint_groups[source] = msg
        self._joint_received_monotonic[source] = time.monotonic()

    def _publish_joint_states(self):
        now_monotonic = time.monotonic()
        stale_after_s = float(self.get_parameter("joint_stale_after_s").value)
        if stale_after_s <= 0.0:
            self.get_logger().error("joint_stale_after_s must be positive")
            return

        stale = stale_sources(
            self._joint_received_monotonic,
            VENDOR_JOINT_TOPICS,
            now_monotonic=now_monotonic,
            stale_after_s=stale_after_s,
        )
        fresh = not stale
        self.pub_joints_fresh.publish(Bool(data=fresh))
        if not fresh:
            short_names = ", ".join(topic.rsplit("/", 1)[-1] for topic in stale)
            self.pub_joints_text.publish(
                String(data=f"JOINT DATA NOT FRESH | missing/stale: {short_names}")
            )
            return

        joints = {}
        stamps = []
        for source in VENDOR_JOINT_TOPICS:
            msg = self._joint_groups[source]
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
                msg.header.stamp.nanosec
            )
            if stamp_ns > 0:
                stamps.append(stamp_ns)
            for i, name in enumerate(msg.name):
                joints[name] = (
                    msg.position[i] if i < len(msg.position) else math.nan,
                    msg.velocity[i] if i < len(msg.velocity) else math.nan,
                    msg.effort[i] if i < len(msg.effort) else math.nan,
                )

        merged = JointState()
        if stamps:
            oldest_stamp_ns = min(stamps)
            merged.header.stamp.sec = oldest_stamp_ns // 1_000_000_000
            merged.header.stamp.nanosec = oldest_stamp_ns % 1_000_000_000
        else:
            merged.header.stamp = self.get_clock().now().to_msg()
        merged.name = list(joints.keys())
        merged.position = [value[0] for value in joints.values()]
        merged.velocity = [value[1] for value in joints.values()]
        merged.effort = [value[2] for value in joints.values()]
        self.pub_joints.publish(merged)
        self.pub_joints_text.publish(
            String(data=f"JOINT DATA FRESH | {len(joints)} joints at bounded publish rate")
        )

    # ---- chrony ------------------------------------------------------------
    def _poll_ntp(self):
        if self._ntp_future is None:
            self._ntp_future = self._submit_ntp_probe()
            return
        if not self._ntp_future.done():
            return

        result = self._ntp_future.result()
        self.pub_offset.publish(Float64(data=result.offset_ms))
        self.pub_skew.publish(Float64(data=result.skew_ppm))
        self.pub_disp.publish(Float64(data=result.root_dispersion_ms))
        self.pub_qualified.publish(Bool(data=result.utc_qualified))
        self.pub_ntp_gate.publish(Bool(data=result.gate_pass))
        if result.error:
            ntp_text = f"NTP UNAVAILABLE | {result.error}"
        else:
            ntp_text = (
                f"NTP world-clock offset = {result.offset_ms:+.3f} ms | "
                f"root dispersion = {result.root_dispersion_ms:.3f} ms | "
                f"skew = {result.skew_ppm:.3f} ppm"
            )
        self.pub_ntp_text.publish(String(data=ntp_text))
        if result.error:
            self.get_logger().warn(
                f"chrony probe unavailable: {result.error}", throttle_duration_sec=30
            )
        self._ntp_future = self._submit_ntp_probe()

    def _submit_ntp_probe(self):
        max_offset_ms = float(self.get_parameter("ntp_max_offset_ms").value)
        max_skew_ppm = float(self.get_parameter("ntp_max_skew_ppm").value)
        return self._probe_pool.submit(
            probe_ntp,
            max_offset_ms=max_offset_ms,
            max_skew_ppm=max_skew_ppm,
        )

    # ---- vendor process-manager state -------------------------------------
    def _poll_pm(self):
        unit = str(self.get_parameter("agibot_pm_unit").value)
        if self._pm_future is None:
            self._pm_future = self._probe_pool.submit(probe_systemd_service, unit)
            return
        if not self._pm_future.done():
            return

        result = self._pm_future.result()
        self.pub_pm_active.publish(Bool(data=result.active))
        label = f"{unit}: {result.state}"
        if result.error:
            label += f" | {result.error}"
        self.pub_pm_text.publish(String(data=label))
        self._pm_future = self._probe_pool.submit(probe_systemd_service, unit)

    def _poll_hdu(self):
        unit = str(self.get_parameter("hdu_runtime_unit").value)
        if self._hdu_future is None:
            self._hdu_future = self._probe_pool.submit(probe_systemd_service, unit)
            return
        if not self._hdu_future.done():
            return

        result = self._hdu_future.result()
        self.pub_hdu_active.publish(Bool(data=result.active))
        label = f"HDU runtime {unit}: {result.state}"
        if result.error:
            label += f" | {result.error}"
        self.pub_hdu_text.publish(String(data=label))
        self._hdu_future = self._probe_pool.submit(probe_systemd_service, unit)

    # ---- pelvis pose -------------------------------------------------------
    def _poll_pelvis(self):
        ref = str(self.get_parameter("reference_frame").value)
        pelvis = str(self.get_parameter("pelvis_frame").value)
        try:
            tf = self._tf_buffer.lookup_transform(ref, pelvis, Time())
        except Exception as exc:  # noqa: BLE001 - surface the error in the UI
            self._set_tf_unready(f"TF lookup {ref} -> {pelvis} failed: {exc}")
            return

        try:
            age_s = timestamp_age_s(
                self.get_clock().now().nanoseconds,
                tf.header.stamp.sec,
                tf.header.stamp.nanosec,
            )
        except ValueError as exc:
            self._set_tf_unready(
                f"TF {ref} -> {pelvis} is not live: {exc}"
            )
            return
        stale_after_s = float(self.get_parameter("tf_stale_after_s").value)
        if age_s < -0.1 or age_s > stale_after_s:
            self._set_tf_unready(
                f"TF {ref} -> {pelvis} is stale/future-dated: age={age_s:+.3f} s"
            )
            return

        self.pub_tf_ready.publish(Bool(data=True))
        self._publish_robot_description(tf_ready=True)
        pose = PoseStamped()
        pose.header = tf.header
        t = tf.transform.translation
        q = tf.transform.rotation
        pose.pose.position.x = t.x
        pose.pose.position.y = t.y
        pose.pose.position.z = t.z
        pose.pose.orientation = q
        self.pub_pelvis.publish(pose)

        roll, pitch, yaw = _quat_to_rpy_deg(q.x, q.y, q.z, q.w)
        self.pub_pelvis_text.publish(
            String(
                data=(
                    f"{ref} -> {pelvis} | "
                    f"pos [m] x={t.x:+.3f} y={t.y:+.3f} z={t.z:+.3f} | "
                    f"quat x={q.x:+.4f} y={q.y:+.4f} z={q.z:+.4f} w={q.w:+.4f} | "
                    f"rpy [deg] r={roll:+.1f} p={pitch:+.1f} y={yaw:+.1f}"
                )
            )
        )

    def _set_tf_unready(self, detail: str):
        self.pub_tf_ready.publish(Bool(data=False))
        self.pub_pelvis_text.publish(String(data=detail))
        self._publish_robot_description(tf_ready=False)

    def _publish_robot_description(self, *, tf_ready: bool):
        show_robot = bool(tf_ready and self._robot_urdf_xml)
        now = time.monotonic()
        publish_period_s = float(
            self.get_parameter("robot_description_publish_period_s").value
        )
        state_changed = show_robot != self._robot_description_visible
        refresh_due = (
            show_robot
            and now - self._last_robot_description_publish_monotonic
            >= publish_period_s
        )
        if not state_changed and not refresh_due:
            return
        payload = self._robot_urdf_xml if show_robot else ""
        self.pub_robot_description.publish(String(data=payload or ""))
        self._robot_description_visible = show_robot
        self._last_robot_description_publish_monotonic = now

    # ---- irreversible-from-UI vendor software E-stop ----------------------
    def _poll_estop_backend(self):
        """Expose the HOPE proxy only while the live vendor RPC is matched.

        This intentionally avoids a clickable Foxglove control when ROS graph
        discovery contains no callable vendor emergency endpoint. The service
        is removed again if the vendor stack disappears.
        """

        client = self._vendor_estop_client
        ready = bool(client is not None and client.service_is_ready())
        self.pub_estop_ready.publish(Bool(data=ready))
        if ready:
            detail = "VENDOR E-STOP RPC READY | Foxglove control enabled"
        elif RosRpcWrapper is None:
            detail = "E-STOP UNAVAILABLE | ros2_plugin_proto is not installed"
        else:
            detail = "E-STOP UNAVAILABLE | waiting for live vendor emergency RPC"
        self.pub_estop_text.publish(String(data=detail))

        if ready != self._last_estop_backend_ready:
            if ready:
                self.get_logger().info(detail)
            else:
                self.get_logger().warn(detail)
            self._last_estop_backend_ready = ready

        with self._estop_service_lock:
            if ready and self._estop_proxy_service is None:
                self._estop_proxy_service = self.create_service(
                    Trigger,
                    "/hope/safety/trigger_estop",
                    self._trigger_estop,
                    callback_group=self._vendor_callback_group,
                )
            elif (
                not ready
                and self._estop_proxy_service is not None
                and not self._estop_call_in_progress
            ):
                self.destroy_service(self._estop_proxy_service)
                self._estop_proxy_service = None

    def _trigger_estop(self, _request, response):
        """Assert the A3 software E-stop; this service cannot clear it."""

        with self._estop_service_lock:
            if self._estop_call_in_progress:
                response.success = False
                response.message = "A3 vendor E-stop call already in progress"
                return response
            # The readiness timer will retain the proxy handle until this call
            # completes, but the lock is not held across vendor I/O. Other
            # monitor timers therefore continue publishing during the call.
            self._estop_call_in_progress = True
        try:
            return self._execute_trigger_estop(response)
        finally:
            with self._estop_service_lock:
                self._estop_call_in_progress = False

    def _execute_trigger_estop(self, response):
        client = self._vendor_estop_client
        if client is None or RosRpcWrapper is None:
            response.success = False
            response.message = (
                "A3 vendor ros2_plugin_proto is unavailable; E-stop was NOT sent"
            )
            return response
        if not client.wait_for_service(timeout_sec=0.5):
            response.success = False
            response.message = "A3 vendor emergency service unavailable; E-stop was NOT sent"
            return response

        trace_id = f"hope-foxglove-{uuid.uuid4()}"
        vendor_request = RosRpcWrapper.Request()
        vendor_request.serialization_type = "pb"
        vendor_request.context = ["aimdk.protocol.EmergencyCommandReq"]
        vendor_request.data = list(
            build_software_estop_request(time.time_ns(), trace_id)
        )
        try:
            vendor_response = client.call(vendor_request, timeout_sec=2.0)
        except Exception as exc:  # noqa: BLE001 - report the safety-call failure
            response.success = False
            response.message = f"A3 vendor E-stop call failed: {exc}"
            return response

        if vendor_response is None or int(vendor_response.code) != 0:
            code = "no response" if vendor_response is None else vendor_response.code
            response.success = False
            response.message = f"A3 vendor rejected E-stop wrapper call: {code}"
            return response

        if vendor_response.serialization_type != "pb":
            response.success = False
            response.message = (
                "A3 vendor E-stop response used unexpected serialization type "
                f"{vendor_response.serialization_type!r}"
            )
            return response
        try:
            application_code, application_message = decode_software_estop_response(
                bytes(vendor_response.data)
            )
        except ValueError as exc:
            response.success = False
            response.message = f"A3 vendor E-stop response could not be decoded: {exc}"
            return response
        if application_code != 0:
            detail = application_message or "no vendor detail"
            response.success = False
            response.message = (
                "A3 vendor rejected E-stop application request: "
                f"code={application_code}, msg={detail}"
            )
            return response

        response.success = True
        response.message = (
            "A3 vendor software E-stop asserted; use the approved local recovery "
            "procedure to inspect and reset"
        )
        return response

    def stop_workers(self):
        self._probe_pool.shutdown(wait=False, cancel_futures=True)


def _quat_to_rpy_deg(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    rclpy.init()
    node = HopeMonitor()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.stop_workers()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
