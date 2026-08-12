#!/usr/bin/env python3
"""HOPE A3 monitoring and narrowly scoped safety publisher for Foxglove.

Reads `chronyc tracking`, local systemd state, one timestamped IMU topic, the
pelvis pose/TF, runner acknowledgments, and the final
base-pose stream. The E-stop path can assert (but never clear) the A3 vendor
software latch and can independently request native Runner PASSIVE. Four
imported `/hope/control/*` services remain as legacy compatibility internals,
but the integrated bridges do not expose them. Motive/NatNet network
acquisition remains outside this process.
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
  /hope/system/cpu_top_process   std_msgs/String  largest per-process CPU delta
  /hope/vendor/agibot_pm_active  std_msgs/Bool     local HDU systemd unit state
  /hope/system/hdu_active    std_msgs/Bool     Runner HDU observer unit state
  /hope/vendor/tf_ready          std_msgs/Bool     fresh reference->pelvis TF
  /hope/safety/estop_ready       std_msgs/Bool     at least one stop path callable
  /hope/safety/estop_full_ready  std_msgs/Bool     vendor + Runner paths callable
  /hope/safety/estop_latched     std_msgs/Bool     persistent assert-only latch
  /hope/pelvis/pose             geometry_msgs/PoseStamped  reference->pelvis TF
  /hope/pelvis/text             std_msgs/String   human-readable pose (or TF error)
  /hope/pelvis/marker           visualization_msgs/Marker  world-pose point/label
  /hope/pelvis/tf               tf2_msgs/TFMessage sanitized reference->pelvis TF

Services:
  /hope/safety/trigger_estop    std_srvs/Trigger  vendor E-stop + native Runner passive
  /hope/control/enter_prepare  std_srvs/Trigger  legacy PD_STAND + calibration workflow
  /hope/control/enter_policy   std_srvs/Trigger  legacy MOTION compatibility request
  /hope/control/exit_policy     std_srvs/Trigger  return MOTION to PD_STAND
  /hope/control/enter_passive   std_srvs/Trigger  press stock runner's PASSIVE key
"""

from concurrent.futures import Future, ThreadPoolExecutor
import math
import os
from pathlib import Path
import threading
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

try:
    from ros2_plugin_proto.srv import RosRpcWrapper
except ImportError:  # Endpoint stays available; only the vendor RPC is degraded.
    RosRpcWrapper = None

from hope_monitor_core import (
    NtpProbeResult,
    ServiceProbeResult,
    build_software_estop_request,
    combine_estop_results,
    cpu_load_percent,
    decode_software_estop_response,
    estop_backend_status,
    message_latency_ms,
    parse_calibration_service_sha,
    parse_proc_stat_cpu,
    parse_process_stat,
    probe_ntp,
    probe_systemd_service,
    timestamp_age_s,
    top_process_cpu_load,
)

RUNNER_MODE_PASSIVE = 0
RUNNER_MODE_PD_STAND = 1
RUNNER_MODE_SHADOW = 2
RUNNER_MODE_MOTION = 3
RUNNER_MODE_IDLE = 6
RUNNER_MODE_STOPPED = 7
RUNNER_MODE_STARTING = 8
RUNNER_MODE_NAMES = {
    RUNNER_MODE_PASSIVE: "PASSIVE",
    RUNNER_MODE_PD_STAND: "PD_STAND",
    RUNNER_MODE_SHADOW: "SHADOW",
    RUNNER_MODE_MOTION: "MOTION",
    4: "REFERENCE_PLAYBACK",
    5: "SERVE",
    RUNNER_MODE_IDLE: "IDLE",
    RUNNER_MODE_STOPPED: "STOPPED",
    RUNNER_MODE_STARTING: "STARTING",
}
MODE_COMMAND_PASSIVE = 0
MODE_COMMAND_PD_STAND = 1
MODE_COMMAND_MOTION = 2

class HopeMonitor(Node):
    def __init__(self):
        super().__init__("hope_monitor")
        self.declare_parameter("period_s", 1.0)
        self.declare_parameter("pelvis_frame", "pelvis_link")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("ntp_max_offset_ms", 10.0)
        self.declare_parameter("ntp_max_skew_ppm", 5.0)
        self.declare_parameter(
            "message_latency_topic", "/ros2/body_drive/pelvis_imu/data"
        )
        self.declare_parameter("message_latency_publish_hz", 20.0)
        self.declare_parameter("message_latency_stale_after_s", 0.5)
        self.declare_parameter("agibot_pm_unit", "agibot_pm.service")
        self.declare_parameter("hdu_runtime_unit", "hope-observer.service")
        self.declare_parameter("cpu_publish_period_s", 1.0)
        self.declare_parameter("tf_stale_after_s", 0.5)
        self.declare_parameter(
            "mode_command_topic", "/hope/runner/mode_command"
        )
        self.declare_parameter("mode_state_topic", "/hope/runner/mode_state")
        self.declare_parameter(
            "runner_estop_service", "/hope/runner/emergency_stop"
        )
        self.declare_parameter("runner_state_stale_after_s", 2.5)
        self.declare_parameter("base_pose_topic", "/a3/base_pose_flat")
        self.declare_parameter("base_pose_stale_after_s", 1.0)
        self.declare_parameter(
            "pelvis_pose_topic", "/a3/mocap/pelvis_pose"
        )
        self.declare_parameter(
            "calibration_service", "/a3/calibration/recompute_p1"
        )
        self.declare_parameter("calibration_timeout_s", 25.0)
        self.declare_parameter(
            "estop_latch_path", "/var/lib/hope-monitor/estop-latched"
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
        self.pub_cpu_top_process = self.create_publisher(
            String, "/hope/system/cpu_top_process", 10
        )
        self.pub_pm_active = self.create_publisher(
            Bool, "/hope/vendor/agibot_pm_active", 10
        )
        self.pub_pm_text = self.create_publisher(
            String, "/hope/vendor/agibot_pm_text", 10
        )
        self.pub_hdu_active = self.create_publisher(
            Bool, "/hope/system/hdu_active", 10
        )
        self.pub_hdu_text = self.create_publisher(
            String, "/hope/system/hdu_text", 10
        )
        self.pub_tf_ready = self.create_publisher(
            Bool, "/hope/vendor/tf_ready", 10
        )
        self.pub_estop_ready = self.create_publisher(
            Bool, "/hope/safety/estop_ready", 10
        )
        self.pub_estop_full_ready = self.create_publisher(
            Bool, "/hope/safety/estop_full_ready", 10
        )
        self.pub_estop_text = self.create_publisher(
            String, "/hope/safety/estop_text", 10
        )
        self.pub_estop_latched = self.create_publisher(
            Bool, "/hope/safety/estop_latched", 10
        )
        self.pub_pelvis = self.create_publisher(PoseStamped, "/hope/pelvis/pose", 10)
        self.pub_pelvis_text = self.create_publisher(String, "/hope/pelvis/text", 10)
        self.pub_pelvis_marker = self.create_publisher(
            Marker, "/hope/pelvis/marker", 10
        )
        self.pub_pelvis_tf = self.create_publisher(
            TFMessage, "/hope/pelvis/tf", 10
        )
        self.pub_runner_ready = self.create_publisher(
            Bool, "/hope/control/runner_ready", 10
        )
        self.pub_calibration_ready = self.create_publisher(
            Bool, "/hope/control/calibration_ready", 10
        )
        self.pub_control_text = self.create_publisher(
            String, "/hope/control/state_text", 10
        )
        self._mode_command_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("mode_command_topic").value),
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._pelvis_lock = threading.Lock()
        self._mocap_pelvis_pose = None
        self._mocap_pelvis_received_monotonic = None
        latency_topic = str(self.get_parameter("message_latency_topic").value)
        self._message_latency_topic = latency_topic
        self._latest_message_latency_ms = None
        self._message_received_monotonic = None
        self._previous_cpu_times = None
        self._previous_process_cpu_times = {}
        self.create_subscription(
            Imu,
            latency_topic,
            self._on_latency_message,
            qos_profile_sensor_data,
        )
        self._vendor_callback_group = ReentrantCallbackGroup()
        # State transitions are serialized so an older PREPARE cannot publish
        # PD_STAND after a newer PASSIVE/exit request. E-stop remains on the
        # independent reentrant group above and is never delayed by this lock.
        self._mode_callback_group = MutuallyExclusiveCallbackGroup()
        self._calibration_callback_group = ReentrantCallbackGroup()
        self._calibration_client = self.create_client(
            Trigger,
            str(self.get_parameter("calibration_service").value),
            callback_group=self._calibration_callback_group,
        )
        self._vendor_estop_client = None
        self._runner_estop_client = self.create_client(
            Trigger,
            str(self.get_parameter("runner_estop_service").value),
            callback_group=self._vendor_callback_group,
        )
        self._estop_service_lock = threading.Lock()
        self._estop_call_in_progress = False
        self._estop_latch_path = Path(
            str(self.get_parameter("estop_latch_path").value)
        )
        self._control_estop_latched = self._estop_latch_path.exists()
        self._last_estop_backend_ready = None
        if RosRpcWrapper is not None:
            self._vendor_estop_client = self.create_client(
                RosRpcWrapper,
                "/aimdk_2Eprotocol_2EHalEmergencyService/SetEmergencyCommand",
                qos_profile=qos_profile_system_default,
                callback_group=self._vendor_callback_group,
            )

        self._control_lock = threading.Lock()
        self._runner_state = None
        self._runner_state_received_monotonic = None
        self._base_flat_valid = False
        self._base_flat_calibration_id = 0
        self._base_flat_received_monotonic = None
        self._mode_command_sequence = max(1, time.time_ns() // 1_000_000)
        self._prepare_request_sequence = 0
        self._prepare_waiting_for_stand = False
        self._prepare_requires_calibration = False
        self._calibration_generation = 0
        self._calibration_future = None
        self._session_calibration_sha = ""
        self._control_detail = "RUNNER UNAVAILABLE | waiting for mode state"
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("mode_state_topic").value),
            self._on_runner_mode_state,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("base_pose_topic").value),
            self._on_base_pose_flat,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pelvis_pose_topic").value),
            self._on_mocap_pelvis_pose,
            qos_profile_sensor_data,
        )
        self.create_service(
            Trigger,
            "/hope/control/enter_prepare",
            self._enter_prepare,
            callback_group=self._mode_callback_group,
        )
        self.create_service(
            Trigger,
            "/hope/control/enter_policy",
            self._enter_policy,
            callback_group=self._mode_callback_group,
        )
        self.create_service(
            Trigger,
            "/hope/control/exit_policy",
            self._exit_policy,
            callback_group=self._mode_callback_group,
        )
        self.create_service(
            Trigger,
            "/hope/control/enter_passive",
            self._enter_passive,
            callback_group=self._mode_callback_group,
        )
        # Keep the assert-only endpoint present regardless of backend telemetry.
        # Readiness is audit information; it must never make the emergency
        # request button disappear exactly when a backend is degraded.
        self._estop_proxy_service = self.create_service(
            Trigger,
            "/hope/safety/trigger_estop",
            self._trigger_estop,
            callback_group=self._vendor_callback_group,
        )

        period = float(self.get_parameter("period_s").value)
        latency_publish_hz = float(
            self.get_parameter("message_latency_publish_hz").value
        )
        cpu_publish_period_s = float(
            self.get_parameter("cpu_publish_period_s").value
        )
        if period <= 0.0:
            raise ValueError("period_s must be positive")
        if latency_publish_hz <= 0.0:
            raise ValueError("message_latency_publish_hz must be positive")
        if cpu_publish_period_s <= 0.0:
            raise ValueError("cpu_publish_period_s must be positive")
        if float(self.get_parameter("tf_stale_after_s").value) <= 0.0:
            raise ValueError("tf_stale_after_s must be positive")
        if float(self.get_parameter("calibration_timeout_s").value) <= 0.0:
            raise ValueError("calibration_timeout_s must be positive")
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
        self.create_timer(0.2, self._poll_control_state)
        self.create_timer(0.2, self._poll_pelvis)
        self.create_timer(1.0 / latency_publish_hz, self._publish_message_latency)

    # ---- CPU load ----------------------------------------------------------
    @staticmethod
    def _read_process_cpu_times():
        result = {}
        for entry in Path("/proc").iterdir():
            if not entry.name.isdecimal():
                continue
            try:
                sample = parse_process_stat(
                    (entry / "stat").read_text(encoding="utf-8")
                )
            except (OSError, UnicodeError, ValueError):
                continue
            result[sample.pid] = sample
        return result

    def _poll_cpu(self):
        try:
            current = parse_proc_stat_cpu(
                Path("/proc/stat").read_text(encoding="utf-8")
            )
            process_current = self._read_process_cpu_times()
            previous = self._previous_cpu_times
            process_previous = self._previous_process_cpu_times
            self._previous_cpu_times = current
            self._previous_process_cpu_times = process_current
            if previous is None:
                self.pub_cpu_text.publish(String(data="CPU LOAD WARMING UP"))
                self.pub_cpu_top_process.publish(
                    String(data="TOP CPU PROCESS WARMING UP")
                )
                return
            load = cpu_load_percent(previous, current)
            top_process = top_process_cpu_load(
                process_previous,
                process_current,
                total_cpu_delta=current.total - previous.total,
                cpu_count=os.cpu_count() or 1,
            )
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
        if top_process is None:
            top_text = "TOP CPU PROCESS UNAVAILABLE"
        else:
            top_text = (
                f"{top_process.name} pid={top_process.pid} "
                f"core={top_process.core_percent:.1f}% "
                f"system={top_process.system_percent:.1f}%"
            )
        self.pub_cpu_top_process.publish(String(data=top_text))

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
    def _on_mocap_pelvis_pose(self, message: PoseStamped):
        p = message.pose.position
        q = message.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not all(math.isfinite(value) for value in values) or not 0.5 <= norm <= 1.5:
            self.get_logger().warning(
                "rejected malformed authoritative pelvis pose",
                throttle_duration_sec=5.0,
            )
            return
        with self._pelvis_lock:
            self._mocap_pelvis_pose = message
            self._mocap_pelvis_received_monotonic = time.monotonic()

    def _poll_pelvis(self):
        ref = str(self.get_parameter("reference_frame").value)
        pelvis = str(self.get_parameter("pelvis_frame").value)
        stale_after_s = float(self.get_parameter("tf_stale_after_s").value)
        now_monotonic = time.monotonic()
        with self._pelvis_lock:
            pose = self._mocap_pelvis_pose
            received = self._mocap_pelvis_received_monotonic
        source = "authoritative mocap pelvis pose"
        pose_error = "no authoritative mocap pelvis pose"
        if pose is not None and received is not None:
            if pose.header.frame_id != ref:
                pose_error = (
                    f"pelvis pose frame {pose.header.frame_id!r} does not match {ref!r}"
                )
                pose = None
            elif now_monotonic - received > stale_after_s:
                pose_error = "authoritative mocap pelvis pose is stale"
                pose = None
            else:
                try:
                    age_s = timestamp_age_s(
                        self.get_clock().now().nanoseconds,
                        pose.header.stamp.sec,
                        pose.header.stamp.nanosec,
                    )
                except ValueError as exc:
                    pose_error = f"authoritative pelvis timestamp is invalid: {exc}"
                    pose = None
                else:
                    if age_s < -0.1 or age_s > stale_after_s:
                        pose_error = (
                            "authoritative mocap pelvis pose is stale/future-dated: "
                            f"age={age_s:+.3f} s"
                        )
                        pose = None

        if pose is None:
            source = "existing TF"
            try:
                tf = self._tf_buffer.lookup_transform(ref, pelvis, Time())
                age_s = timestamp_age_s(
                    self.get_clock().now().nanoseconds,
                    tf.header.stamp.sec,
                    tf.header.stamp.nanosec,
                )
                if age_s < -0.1 or age_s > stale_after_s:
                    raise ValueError(f"age={age_s:+.3f} s")
            except Exception as exc:  # noqa: BLE001 - surface both sources
                self._set_tf_unready(
                    f"PELVIS UNAVAILABLE | {pose_error}; "
                    f"TF lookup {ref} -> {pelvis} failed: {exc}"
                )
                return
            pose = PoseStamped()
            pose.header = tf.header
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation

        self.pub_tf_ready.publish(Bool(data=True))
        self.pub_pelvis.publish(pose)

        root_tf = TransformStamped()
        root_tf.header = pose.header
        root_tf.child_frame_id = pelvis
        root_tf.transform.translation.x = pose.pose.position.x
        root_tf.transform.translation.y = pose.pose.position.y
        root_tf.transform.translation.z = pose.pose.position.z
        root_tf.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(root_tf)
        # Foxglove receives only this sanitized transform topic. The raw /tf
        # and /tf_static topics stay outside both WebSocket allowlists, so the
        # 3D panel can show world -> pelvis_link without exposing every robot
        # link/frame.
        self.pub_pelvis_tf.publish(TFMessage(transforms=[root_tf]))

        t = pose.pose.position
        q = pose.pose.orientation
        roll, pitch, yaw = _quat_to_rpy_deg(q.x, q.y, q.z, q.w)
        self.pub_pelvis_text.publish(
            String(
                data=(
                    f"{ref} -> {pelvis} | {source} | "
                    f"pos [m] x={t.x:+.3f} y={t.y:+.3f} z={t.z:+.3f} | "
                    f"quat x={q.x:+.4f} y={q.y:+.4f} z={q.z:+.4f} w={q.w:+.4f} | "
                    f"rpy [deg] r={roll:+.1f} p={pitch:+.1f} y={yaw:+.1f}"
                )
            )
        )
        self._publish_pelvis_markers(
            pose,
            pelvis,
            t.x,
            t.y,
            t.z,
            roll,
            pitch,
            yaw,
        )

    def _publish_pelvis_markers(
        self, pose, pelvis, x, y, z, roll, pitch, yaw
    ):
        point = Marker()
        point.header = pose.header
        point.ns = "hope_pelvis"
        point.id = 0
        point.type = Marker.SPHERE
        point.action = Marker.ADD
        point.pose = pose.pose
        point.scale.x = 0.08
        point.scale.y = 0.08
        point.scale.z = 0.08
        point.color.r = 0.10
        point.color.g = 0.85
        point.color.b = 1.0
        point.color.a = 1.0
        point.lifetime.nanosec = 500_000_000
        point.frame_locked = True
        self.pub_pelvis_marker.publish(point)

        label = Marker()
        label.header = pose.header
        label.ns = "hope_pelvis"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = z + 0.18
        label.pose.orientation.w = 1.0
        label.scale.z = 0.10
        label.color.r = 1.0
        label.color.g = 0.78
        label.color.b = 0.40
        label.color.a = 1.0
        label.lifetime.nanosec = 500_000_000
        label.frame_locked = True
        label.text = (
            f"{pelvis}\n"
            f"x {x:+.3f}  y {y:+.3f}  z {z:+.3f}\n"
            f"rpy {roll:+.1f} deg  {pitch:+.1f} deg  {yaw:+.1f} deg"
        )
        self.pub_pelvis_marker.publish(label)

    def _set_tf_unready(self, detail: str):
        self.pub_tf_ready.publish(Bool(data=False))
        self.pub_pelvis_text.publish(String(data=detail))

    # ---- legacy 8-double adapter compatibility; not bridge-exposed ---------
    @staticmethod
    def _exact_integer(value, *, minimum=0, maximum=(1 << 52)):
        number = float(value)
        if not math.isfinite(number) or number < minimum or number > maximum:
            raise ValueError("integer field is out of range")
        integer = int(number)
        if float(integer) != number:
            raise ValueError("integer field is not exact")
        return integer

    def _on_runner_mode_state(self, message):
        values = list(message.data)
        try:
            if len(values) != 8 or values[0] != 1.0:
                raise ValueError("mode-state schema/size mismatch")
            mode = self._exact_integer(values[1], maximum=RUNNER_MODE_STARTING)
            pd_ready = self._exact_integer(values[2], maximum=1)
            received_sequence = self._exact_integer(values[3])
            applied_sequence = self._exact_integer(values[4])
            result = self._exact_integer(
                abs(values[5]), maximum=3
            ) * (-1 if values[5] < 0.0 else 1)
            base_ready = self._exact_integer(values[6], maximum=1)
            command_fault = self._exact_integer(values[7], maximum=1)
        except (TypeError, ValueError):
            self.get_logger().warning(
                "rejected malformed /hope/runner/mode_state",
                throttle_duration_sec=5.0,
            )
            return
        with self._control_lock:
            self._runner_state = {
                "mode": mode,
                "pd_ready": bool(pd_ready),
                "received_sequence": received_sequence,
                "applied_sequence": applied_sequence,
                "result": result,
                "base_ready": bool(base_ready),
                "command_fault": bool(command_fault),
            }
            self._runner_state_received_monotonic = time.monotonic()

    def _on_base_pose_flat(self, message):
        values = list(message.data)
        valid = False
        calibration_id = 0
        try:
            if len(values) >= 16 and values[0] == 2.0 and values[1] == 1.0:
                calibration_id = self._exact_integer(
                    values[14], minimum=1
                )
                valid = True
        except (TypeError, ValueError):
            valid = False
            calibration_id = 0
        with self._control_lock:
            self._base_flat_valid = valid
            self._base_flat_calibration_id = calibration_id
            self._base_flat_received_monotonic = time.monotonic()

    def _runner_snapshot(self):
        with self._control_lock:
            state = None if self._runner_state is None else dict(self._runner_state)
            received = self._runner_state_received_monotonic
        stale_after = float(
            self.get_parameter("runner_state_stale_after_s").value
        )
        fresh = (
            state is not None
            and received is not None
            and time.monotonic() - received <= stale_after
        )
        return state, fresh

    def _publish_mode_command(self, command_code):
        with self._control_lock:
            if self._control_estop_latched:
                return 0
            self._mode_command_sequence += 1
            sequence = self._mode_command_sequence
            message = Float64MultiArray()
            message.data = [1.0, float(sequence), float(command_code)]
            self._mode_command_pub.publish(message)
        return sequence

    def _base_matches_session_calibration(self):
        with self._control_lock:
            receipt_sha = self._session_calibration_sha
            valid = self._base_flat_valid
            calibration_id = self._base_flat_calibration_id
            received = self._base_flat_received_monotonic
        if not receipt_sha or not valid or received is None:
            return False
        stale_after = float(
            self.get_parameter("base_pose_stale_after_s").value
        )
        if time.monotonic() - received > stale_after:
            return False
        return calibration_id == int(receipt_sha[:13], 16)

    def _start_calibration(self, expected_generation, expected_request_sequence):
        with self._control_lock:
            if (
                self._calibration_generation != expected_generation
                or not self._prepare_waiting_for_stand
                or not self._prepare_requires_calibration
                or self._prepare_request_sequence != expected_request_sequence
                or self._calibration_future is not None
            ):
                return
            if not self._calibration_client.service_is_ready():
                self._prepare_waiting_for_stand = False
                self._prepare_requires_calibration = False
                self._prepare_request_sequence = 0
                self._session_calibration_sha = ""
                self._control_detail = (
                    "CALIBRATION START FAILED | laptop service unavailable"
                )
                return
            generation = expected_generation
            try:
                future = self._calibration_client.call_async(Trigger.Request())
            except Exception as exc:  # noqa: BLE001 - surface ROS client failure
                self._prepare_waiting_for_stand = False
                self._prepare_requires_calibration = False
                self._prepare_request_sequence = 0
                self._session_calibration_sha = ""
                self._control_detail = f"CALIBRATION START FAILED | {exc}"
                self.get_logger().error(f"cannot start P1 calibration: {exc}")
                return
            self._prepare_waiting_for_stand = False
            self._prepare_requires_calibration = False
            self._prepare_request_sequence = 0
            self._control_detail = "PD_STAND READY | laptop calibration running"
            self._calibration_future = (future, generation, time.monotonic())
        self.get_logger().info(
            "PD_STAND acknowledged; requested laptop 10-marker P1 calibration"
        )

    def _finish_calibration_if_ready(self):
        with self._control_lock:
            record = self._calibration_future
        if record is None:
            return
        future, generation, started_monotonic = record
        if not future.done():
            timeout_s = float(
                self.get_parameter("calibration_timeout_s").value
            )
            if time.monotonic() - started_monotonic <= timeout_s:
                return
            future.cancel()
            with self._control_lock:
                if self._calibration_future != record:
                    return
                self._calibration_future = None
                if generation == self._calibration_generation:
                    self._session_calibration_sha = ""
                    self._control_detail = "CALIBRATION FAILED | laptop service timed out"
            return
        try:
            failure_detail = "laptop calibration returned no response"
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - surface worker failure
            result = None
            failure_detail = str(exc)
        try:
            if result is None:
                raise ValueError(failure_detail)
            if not result.success:
                raise ValueError(result.message or "laptop calibration failed")
            receipt_sha = parse_calibration_service_sha(result.message)
        except (TypeError, ValueError) as exc:
            with self._control_lock:
                if self._calibration_future != record:
                    return
                self._calibration_future = None
                current = generation == self._calibration_generation
                if current:
                    self._session_calibration_sha = ""
                    self._control_detail = f"CALIBRATION FAILED | {exc}"
            if current:
                self.get_logger().error(f"laptop P1 calibration failed: {exc}")
            return
        with self._control_lock:
            if self._calibration_future != record:
                return
            self._calibration_future = None
            if generation != self._calibration_generation:
                return
            self._session_calibration_sha = receipt_sha
            self._control_detail = (
                "LAPTOP CALIBRATION SAVED | waiting for matching "
                "/a3/base_pose_flat"
            )
        self.get_logger().info(
            f"laptop installed approved P1 calibration {receipt_sha}"
        )

    def _cancel_prepare(self, detail, *, clear_calibration=True):
        with self._control_lock:
            self._calibration_generation += 1
            self._prepare_waiting_for_stand = False
            self._prepare_requires_calibration = False
            self._prepare_request_sequence = 0
            if clear_calibration:
                self._session_calibration_sha = ""
            self._control_detail = detail

    def _enter_prepare(self, _request, response):
        if self._control_estop_latched:
            response.success = False
            response.message = "E-stop is latched; use the approved local recovery procedure"
            return response
        state, fresh = self._runner_snapshot()
        if not fresh:
            response.success = False
            response.message = "runner mode state is unavailable; PD_STAND was NOT requested"
            return response
        if state["command_fault"]:
            response.success = False
            response.message = "runner command fault is latched; PD_STAND was NOT requested"
            return response
        if (
            state["received_sequence"] > state["applied_sequence"]
            and state["result"] == 0
        ):
            response.success = False
            response.message = "a runner command is already in progress"
            return response
        if not self._calibration_client.service_is_ready():
            response.success = False
            response.message = (
                "laptop calibration service is unavailable; "
                "PD_STAND was NOT requested"
            )
            return response
        with self._control_lock:
            if self._calibration_future is not None:
                response.success = False
                response.message = "10-marker calibration is already running"
                return response
            self._calibration_generation += 1
            # Every PREPARE starts a new initialization. The prior laptop JSON
            # stays only as rollback until the new fit succeeds; it cannot
            # authorize policy entry for this session.
            self._session_calibration_sha = ""
            self._prepare_waiting_for_stand = True
            self._prepare_requires_calibration = True
            generation = self._calibration_generation
        sequence = self._publish_mode_command(MODE_COMMAND_PD_STAND)
        if sequence == 0:
            self._cancel_prepare("E-STOP LATCHED | PREPARE cancelled")
            response.success = False
            response.message = "E-stop latched before PD_STAND could be requested"
            return response
        with self._control_lock:
            if generation == self._calibration_generation:
                self._prepare_request_sequence = sequence
                self._control_detail = (
                    f"PREPARE REQUESTED seq={sequence} | waiting for PD_STAND readiness"
                )
        response.success = True
        response.message = (
            f"PD_STAND requested (seq={sequence}); this init's 10-marker "
            "calibration will run on the laptop after settle and atomically "
            "replace the laptop JSON"
        )
        return response

    def _enter_policy(self, _request, response):
        if self._control_estop_latched:
            response.success = False
            response.message = "E-stop is latched; MOTION was NOT requested"
            return response
        state, fresh = self._runner_snapshot()
        if not fresh or state is None:
            response.success = False
            response.message = "runner mode state is unavailable; MOTION was NOT requested"
            return response
        if state["command_fault"]:
            response.success = False
            response.message = "runner command fault is latched; MOTION was NOT requested"
            return response
        if state["mode"] != RUNNER_MODE_PD_STAND or not state["pd_ready"]:
            response.success = False
            response.message = "robot is not in settled PD_STAND; MOTION was NOT requested"
            return response
        with self._control_lock:
            session_receipt_sha = self._session_calibration_sha
        if not session_receipt_sha:
            response.success = False
            response.message = (
                "this run has no approved laptop calibration; "
                "run PREPARE again before MOTION"
            )
            return response
        if not self._base_matches_session_calibration():
            response.success = False
            response.message = (
                "approved calibration is not yet present in valid /a3/base_pose_flat; "
                "MOTION was NOT requested"
            )
            return response
        sequence = self._publish_mode_command(MODE_COMMAND_MOTION)
        if sequence == 0:
            response.success = False
            response.message = "E-stop latched before MOTION could be requested"
            return response
        with self._control_lock:
            self._control_detail = f"POLICY REQUESTED seq={sequence} | waiting for runner ack"
        response.success = True
        response.message = f"MOTION/policy requested (seq={sequence})"
        return response

    def _exit_policy(self, _request, response):
        if self._control_estop_latched:
            response.success = False
            response.message = "E-stop is latched; runner transition was NOT requested"
            return response
        _state, fresh = self._runner_snapshot()
        if not fresh:
            response.success = False
            response.message = "runner mode state is unavailable; PD_STAND was NOT requested"
            return response
        self._cancel_prepare(
            "EXIT POLICY REQUESTED | waiting for PD_STAND ack",
            clear_calibration=False,
        )
        sequence = self._publish_mode_command(MODE_COMMAND_PD_STAND)
        if sequence == 0:
            response.success = False
            response.message = "E-stop latched before PD_STAND could be requested"
            return response
        response.success = True
        response.message = f"PD_STAND requested (seq={sequence}); no calibration started"
        return response

    def _enter_passive(self, _request, response):
        if self._control_estop_latched:
            response.success = False
            response.message = "E-stop is latched; managed runner stop owns recovery"
            return response
        _state, fresh = self._runner_snapshot()
        if not fresh:
            response.success = False
            response.message = "runner mode state is unavailable; PASSIVE was NOT requested"
            return response
        self._cancel_prepare("PASSIVE REQUESTED | waiting for runner ack")
        sequence = self._publish_mode_command(MODE_COMMAND_PASSIVE)
        if sequence == 0:
            response.success = False
            response.message = "E-stop latched before PASSIVE could be requested"
            return response
        response.success = True
        response.message = f"PASSIVE requested (seq={sequence})"
        return response

    def _poll_control_state(self):
        self._finish_calibration_if_ready()
        state, fresh = self._runner_snapshot()
        runner_ready = bool(
            fresh
            and state is not None
            and state["mode"] not in {RUNNER_MODE_STOPPED, RUNNER_MODE_STARTING}
            and not state["command_fault"]
        )
        self.pub_runner_ready.publish(Bool(data=runner_ready))
        calibration_ready = self._base_matches_session_calibration()
        self.pub_calibration_ready.publish(Bool(data=calibration_ready))
        if not fresh or state is None:
            detail = "RUNNER UNAVAILABLE | start the approved MDU policy runner"
        else:
            with self._control_lock:
                waiting = self._prepare_waiting_for_stand
                requires_calibration = self._prepare_requires_calibration
                request_sequence = self._prepare_request_sequence
                generation = self._calibration_generation
                calibration_running = self._calibration_future is not None
                configured_detail = self._control_detail
            if (
                waiting
                and state["mode"] == RUNNER_MODE_PD_STAND
                and state["pd_ready"]
                and state["applied_sequence"] >= request_sequence
                and request_sequence > 0
            ):
                if requires_calibration:
                    self._start_calibration(generation, request_sequence)
                    with self._control_lock:
                        calibration_running = self._calibration_future is not None
                        configured_detail = self._control_detail
            elif (
                waiting
                and request_sequence > 0
                and state["received_sequence"] >= request_sequence
                and state["applied_sequence"] < request_sequence
                and state["result"] < 0
            ):
                self._cancel_prepare(
                    f"PREPARE REJECTED | runner result={state['result']}"
                )
                configured_detail = (
                    f"PREPARE REJECTED | runner result={state['result']}"
                )
            mode_name = RUNNER_MODE_NAMES.get(state["mode"], f"MODE_{state['mode']}")
            if state["mode"] == RUNNER_MODE_STOPPED and not state["command_fault"]:
                detail = "STOPPED | ENTER PREPARE starts the approved model21800 runner"
            elif state["mode"] == RUNNER_MODE_STARTING:
                detail = "STARTING | waiting for all six stock runner inputs"
            elif calibration_ready:
                detail = (
                    f"{mode_name} | PD_READY={int(state['pd_ready'])} | "
                    "CALIBRATION READY | policy gate open"
                )
            elif calibration_running:
                detail = f"{mode_name} | 10-MARKER CALIBRATION RUNNING"
            else:
                detail = f"{mode_name} | {configured_detail}"
            if state["result"] < 0:
                detail += f" | last runner request rejected ({state['result']})"
            if state["command_fault"]:
                detail += " | COMMAND FAULT LATCHED"
        self.pub_control_text.publish(String(data=detail))

    # ---- irreversible-from-UI vendor software E-stop ----------------------
    def _persist_estop_latch(self) -> None:
        self._estop_latch_path.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(
            self._estop_latch_path,
            os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_CLOEXEC,
            0o600,
        )
        try:
            os.write(
                descriptor,
                f"asserted_utc_ns={time.time_ns()}\n".encode("ascii"),
            )
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def _poll_estop_backend(self):
        """Publish backend readiness as audit-only operator telemetry.

        Managed model21800 operation intentionally stops ``agibot_pm``, which
        also removes the vendor emergency RPC. In that state the native Runner
        emergency-PASSIVE path remains actionable, but telemetry and the call
        result explicitly identify it as a partial software stop.
        """

        client = self._vendor_estop_client
        vendor_ready = bool(client is not None and client.service_is_ready())
        runner_stop_ready = self._runner_estop_client.service_is_ready()
        with self._control_lock:
            latched = self._control_estop_latched
        status = estop_backend_status(
            vendor_ready=vendor_ready,
            runner_ready=runner_stop_ready,
            latched=latched,
            vendor_protocol_available=RosRpcWrapper is not None,
        )
        ready = status.action_ready
        self.pub_estop_ready.publish(Bool(data=status.action_ready))
        self.pub_estop_full_ready.publish(Bool(data=status.full_ready))
        self.pub_estop_latched.publish(Bool(data=latched))
        detail = status.detail
        self.pub_estop_text.publish(String(data=detail))

        if ready != self._last_estop_backend_ready:
            if ready:
                self.get_logger().info(detail)
            else:
                self.get_logger().warn(detail)
            self._last_estop_backend_ready = ready

    def _trigger_estop(self, _request, response):
        """Assert or reassert the A3 software E-stop; never clear it."""

        with self._estop_service_lock:
            if self._estop_call_in_progress:
                response.success = False
                response.message = "A3 vendor E-stop call already in progress"
                return response
            # Do not hold this lock across vendor I/O. Other monitor timers
            # therefore continue publishing during the call.
            self._estop_call_in_progress = True
        with self._control_lock:
            was_latched = self._control_estop_latched
            if not was_latched:
                self._control_estop_latched = True
                self._calibration_generation += 1
                self._prepare_waiting_for_stand = False
                self._prepare_requires_calibration = False
                self._prepare_request_sequence = 0
                self._session_calibration_sha = ""
            self._control_detail = (
                "E-STOP REASSERTING | local recovery required"
                if was_latched
                else "E-STOP LATCHED | local recovery required"
            )
        self.pub_estop_latched.publish(Bool(data=True))
        persistence_error = ""
        try:
            self._persist_estop_latch()
        except OSError as exc:
            # The in-memory latch and both stop paths still take effect. Surface
            # persistence loss loudly because a service restart could otherwise
            # hide the asserted state.
            self.get_logger().error(f"cannot persist E-stop latch: {exc}")
            persistence_error = str(exc)
        try:
            result = self._execute_trigger_estop(response)
            if was_latched:
                result.message = f"E-STOP REASSERTED | {result.message}"
            if persistence_error:
                result.success = False
                result.message += (
                    "; LOCAL LATCH PERSISTENCE FAILED: "
                    f"{persistence_error}; keep the physical E-stop asserted"
                )
            return result
        finally:
            with self._estop_service_lock:
                self._estop_call_in_progress = False

    def _execute_trigger_estop(self, response):
        deadline = time.monotonic() + 2.7
        runner_future = None
        runner_stopped = False
        runner_detail = "managed runner stop service unavailable"
        if self._runner_estop_client.service_is_ready():
            try:
                # Start the independent command-source removal before the vendor
                # RPC. The vendor path below remains the primary safety action.
                runner_future = self._runner_estop_client.call_async(
                    Trigger.Request()
                )
                runner_detail = "managed runner stop pending"
            except Exception as exc:  # noqa: BLE001 - aggregate both paths
                runner_detail = f"managed runner stop call failed: {exc}"

        vendor_accepted = False
        client = self._vendor_estop_client
        if client is None or RosRpcWrapper is None:
            vendor_detail = "ros2_plugin_proto unavailable"
        elif not client.wait_for_service(timeout_sec=0.5):
            vendor_detail = "vendor emergency service unavailable"
        else:
            trace_id = f"hope-foxglove-{uuid.uuid4()}"
            vendor_request = RosRpcWrapper.Request()
            vendor_request.serialization_type = "pb"
            vendor_request.context = ["aimdk.protocol.EmergencyCommandReq"]
            vendor_request.data = list(
                build_software_estop_request(time.time_ns(), trace_id)
            )
            try:
                vendor_response = client.call(vendor_request, timeout_sec=2.0)
            except Exception as exc:  # noqa: BLE001 - aggregate both paths
                vendor_response = None
                vendor_detail = f"vendor call failed: {exc}"
            else:
                if vendor_response is None:
                    vendor_detail = "vendor returned no response"
                elif int(vendor_response.code) != 0:
                    vendor_detail = (
                        "vendor wrapper rejected request: "
                        f"code={vendor_response.code}"
                    )
                elif vendor_response.serialization_type != "pb":
                    vendor_detail = (
                        "vendor returned unexpected serialization type "
                        f"{vendor_response.serialization_type!r}"
                    )
                else:
                    try:
                        application_code, application_message = (
                            decode_software_estop_response(
                                bytes(vendor_response.data)
                            )
                        )
                    except ValueError as exc:
                        vendor_detail = f"vendor response decode failed: {exc}"
                    else:
                        vendor_accepted = application_code == 0
                        vendor_detail = (
                            "vendor request accepted"
                            if vendor_accepted
                            else (
                                "vendor application rejected request: "
                                f"code={application_code}, "
                                f"msg={application_message or 'no detail'}"
                            )
                        )

        if runner_future is not None:
            while not runner_future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not runner_future.done():
                runner_detail = "managed runner stop timed out"
            else:
                try:
                    runner_response = runner_future.result()
                except Exception as exc:  # noqa: BLE001 - aggregate both paths
                    runner_detail = f"managed runner stop failed: {exc}"
                else:
                    runner_stopped = bool(
                        runner_response is not None and runner_response.success
                    )
                    runner_detail = (
                        "managed runner stopped"
                        if runner_stopped
                        else (
                            runner_response.message
                            if runner_response is not None
                            else "managed runner returned no response"
                        )
                    )

        response.success, response.message = combine_estop_results(
            vendor_accepted=vendor_accepted,
            vendor_detail=vendor_detail,
            runner_stopped=runner_stopped,
            runner_detail=runner_detail,
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
