"""ROS 2 node: ``planner_imitate`` - a fake HITTER planner for bring-up.

TEMPORARY fake planner for sim-to-real bring-up BEFORE mocap / real ball
tracking are ready. It publishes the SAME message the real planner does
(``hope_msgs/RacketCommand`` on ``/racket/command``) using safe, repeatable,
staged strike presets instead of ball prediction. It does NOT play table tennis
and is NOT the final ball planner. Use the DETERMINISTIC (no-dither) policy;
MuJoCo showed dither is unnecessary for stability and hurts the backhand.

It does NOT command hardware directly - it only publishes planner targets. The
downstream WBC controller (not in this workspace yet) consumes ``/racket/command``
behind its own enable/safety gating. For extra safety this node:
  * defaults to ``dry_run:=true`` (logs only, publishes NOTHING), and
  * honours an emergency-stop topic (``/hope/estop`` std_msgs/Bool) -> stand.

The real mocap path (``hope_planner_node``) is left completely untouched; this is
an additional, separately-launched command source.

See imitate_presets.py for the frame convention and the staged bring-up levels,
and PLANNER_IMITATE.md for the run recipes and the safe-first-test checklist.
"""

import csv
import math
import os

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from hope_msgs.msg import RacketCommand

from .imitate_presets import LEVELS, ImitateConfig, ImitatePlanner, SafetyLimits

_SWING_COLOR = {  # RViz marker color per swing type (RGBA)
    "forehand": (0.10, 0.50, 0.95, 0.9),   # blue
    "backhand": (0.95, 0.45, 0.10, 0.9),   # orange
    "stand": (0.6, 0.6, 0.6, 0.6),         # grey
}


class PlannerImitateNode(Node):
    """Publishes staged fake RacketCommands for deployment bring-up."""

    def __init__(self):
        super().__init__("planner_imitate")

        # --- parameters (all overridable via CLI / yaml) ---
        self.declare_parameter("level", 0)                  # bring-up level 0..5
        self.declare_parameter("frame_id", "base_link")     # "base_link" (default) | "world"
        self.declare_parameter("dry_run", True)             # True => log only, publish nothing
        self.declare_parameter("publish_rate_hz", 50.0)     # command/update rate
        self.declare_parameter("strike_period_s", 3.0)      # seconds per fake strike
        self.declare_parameter("normal_speed", 2.5)         # m/s target speed (levels 2,4,5)
        self.declare_parameter("slow_speed", 1.0)           # m/s target speed (levels 1,3)
        self.declare_parameter("elevation_deg", 18.0)       # target vel/normal tilt above horizontal
        # safety
        self.declare_parameter("max_speed", 3.0)            # m/s hard speed ceiling
        self.declare_parameter("max_pos_step", 0.50)        # m, max target-position slew per cycle
        self.declare_parameter("backhand_disabled", False)  # hard-disable backhand swings
        self.declare_parameter("estop_topic", "/hope/estop")
        # world-frame transform (only used when frame_id == "world"); UNMEASURED today
        self.declare_parameter("robot_world_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("robot_world_yaw", 0.0)
        # logging / viz
        self.declare_parameter("csv_path", "")              # "" => no CSV
        self.declare_parameter("publish_markers", True)

        gp = self.get_parameter
        level = int(gp("level").value)
        if level not in LEVELS:
            raise ValueError(f"level must be one of {sorted(LEVELS)}, got {level}")
        frame_id = str(gp("frame_id").value)
        if frame_id not in ("base_link", "world"):
            raise ValueError(f"frame_id must be 'base_link' or 'world', got '{frame_id}'")

        self._dry_run = bool(gp("dry_run").value)
        self._publish_markers = bool(gp("publish_markers").value)
        rate = float(gp("publish_rate_hz").value)

        cfg = ImitateConfig(
            level=level,
            frame_id=frame_id,
            strike_period_s=float(gp("strike_period_s").value),
            normal_speed=float(gp("normal_speed").value),
            slow_speed=float(gp("slow_speed").value),
            elevation_deg=float(gp("elevation_deg").value),
            robot_world_xyz=tuple(float(x) for x in gp("robot_world_xyz").value),
            robot_world_yaw=float(gp("robot_world_yaw").value),
            safety=SafetyLimits(
                max_speed=float(gp("max_speed").value),
                max_pos_step=float(gp("max_pos_step").value),
                backhand_disabled=bool(gp("backhand_disabled").value),
            ),
        )
        self._cfg = cfg
        self.planner = ImitatePlanner(cfg)

        if frame_id == "world" and all(v == 0.0 for v in cfg.robot_world_xyz):
            self.get_logger().warn(
                "frame_id='world' but robot_world_xyz is all zero (UNMEASURED). World-frame "
                "targets will be mis-placed until you set the robot's world pose "
                "(hope_world_frame.yaml: mocap_to_base_link). Prefer frame_id='base_link' for bring-up."
            )

        # QoS matches the real planner (best-effort, depth 1) so this is a drop-in source.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cmd_pub = self.create_publisher(RacketCommand, "/racket/command", qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/planner_imitate/diagnostics", 1)
        if self._publish_markers:
            self.marker_pub = self.create_publisher(MarkerArray, "/planner_imitate/markers", 1)

        # emergency-stop subscription: True -> stand, stop swinging.
        self.create_subscription(Bool, str(gp("estop_topic").value), self._estop_cb, 1)

        # CSV logging
        self._csv_path = str(gp("csv_path").value).strip()
        self._csv_file = None
        self._csv = None
        if self._csv_path:
            os.makedirs(os.path.dirname(os.path.abspath(self._csv_path)), exist_ok=True)
            self._csv_file = open(self._csv_path, "w", newline="")
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow([
                "t", "seq", "level", "level_name", "swing_type", "frame_id",
                "strike_phase", "time_to_strike", "strike_time", "clip_phase",
                "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z",
                "norm_x", "norm_y", "norm_z", "speed", "valid", "clamped",
                "out_of_training_range", "dry_run",
            ])

        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self._last_seq_logged = -1
        self.create_timer(1.0 / max(rate, 1e-3), self._tick)

        self.get_logger().info(
            f"planner_imitate started | level={level} ({LEVELS[level].name}) | frame_id={frame_id} | "
            f"dry_run={self._dry_run} | rate={rate:.0f} Hz | strike_period={cfg.strike_period_s:.1f}s | "
            f"{'PUBLISHING to /racket/command' if not self._dry_run else 'DRY-RUN (no publish)'}"
        )
        self.get_logger().warn(
            "TEMPORARY fake planner (no real ball). Use DETERMINISTIC policy (no dither). "
            "Start at level 0/1 before 5. This is NOT the final ball planner."
        )

    # ----- callbacks -----
    def _estop_cb(self, msg: Bool):
        self.planner.set_estop(bool(msg.data))
        if msg.data:
            self.get_logger().warn("ESTOP engaged -> commanding STAND (valid=False).")

    def _tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        cmd = self.planner.step(t)

        # publish RacketCommand (unless dry-run). Hardware never moves from this node directly.
        if not self._dry_run:
            self.cmd_pub.publish(self._to_msg(cmd))
        if self._publish_markers:
            self.marker_pub.publish(self._to_markers(cmd))
        self._log(cmd)
        self._publish_diag(cmd)

    # ----- helpers -----
    def _to_msg(self, cmd) -> RacketCommand:
        out = RacketCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = cmd.frame_id          # <-- frame name carried on every command
        out.position.x, out.position.y, out.position.z = (float(v) for v in cmd.position)
        out.velocity.x, out.velocity.y, out.velocity.z = (float(v) for v in cmd.velocity)
        out.normal.x, out.normal.y, out.normal.z = (float(v) for v in cmd.normal)
        out.strike_time = 0.0 if math.isnan(cmd.strike_time) else float(cmd.strike_time)
        out.time_to_strike = 0.0 if math.isnan(cmd.time_to_strike) else float(cmd.time_to_strike)
        out.ball_velocity_outgoing.x = 0.0  # no ball model in the fake planner
        out.ball_velocity_outgoing.y = 0.0
        out.ball_velocity_outgoing.z = 0.0
        out.valid = bool(cmd.valid)
        out.clears_net = False              # not modelled (no ball)
        out.bypasses_net_posts = False
        out.predicted_bounces = 0
        return out

    def _to_markers(self, cmd) -> MarkerArray:
        arr = MarkerArray()
        r, g, b, a = _SWING_COLOR.get(cmd.swing_type, _SWING_COLOR["stand"])
        stamp = self.get_clock().now().to_msg()
        # target position sphere
        sph = Marker()
        sph.header.frame_id = cmd.frame_id
        sph.header.stamp = stamp
        sph.ns = "planner_imitate_target"
        sph.id = 0
        sph.type = Marker.SPHERE
        sph.action = Marker.ADD
        sph.pose.position.x, sph.pose.position.y, sph.pose.position.z = (float(v) for v in cmd.position)
        sph.pose.orientation.w = 1.0
        sph.scale.x = sph.scale.y = sph.scale.z = 0.09
        sph.color.r, sph.color.g, sph.color.b, sph.color.a = r, g, b, a
        arr.markers.append(sph)
        # racket-normal arrow (from target along the face normal)
        arr_m = Marker()
        arr_m.header.frame_id = cmd.frame_id
        arr_m.header.stamp = stamp
        arr_m.ns = "planner_imitate_normal"
        arr_m.id = 1
        arr_m.type = Marker.ARROW
        arr_m.action = Marker.ADD
        from geometry_msgs.msg import Point
        p0 = Point(x=float(cmd.position[0]), y=float(cmd.position[1]), z=float(cmd.position[2]))
        p1 = Point(
            x=float(cmd.position[0] + 0.2 * cmd.normal[0]),
            y=float(cmd.position[1] + 0.2 * cmd.normal[1]),
            z=float(cmd.position[2] + 0.2 * cmd.normal[2]),
        )
        arr_m.points = [p0, p1]
        arr_m.scale.x, arr_m.scale.y, arr_m.scale.z = 0.012, 0.025, 0.0
        arr_m.color.r, arr_m.color.g, arr_m.color.b, arr_m.color.a = r, g, b, a
        arr.markers.append(arr_m)
        return arr

    def _log(self, cmd):
        if cmd.seq != self._last_seq_logged:
            # new swing -> one concise console line (per-tick spam avoided)
            self._last_seq_logged = cmd.seq
            speed = float(np.linalg.norm(cmd.velocity))
            self.get_logger().info(
                f"[seq {cmd.seq}] L{cmd.level}:{cmd.level_name} {cmd.swing_type} "
                f"frame={cmd.frame_id} pos=({cmd.position[0]:+.2f},{cmd.position[1]:+.2f},"
                f"{cmd.position[2]:+.2f}) v={speed:.2f} m/s phase={cmd.strike_phase:.2f} "
                f"valid={cmd.valid}{' CLAMPED' if cmd.clamped else ''}"
                f"{' OUT-OF-TRAINING-RANGE' if cmd.out_of_training_range else ''}"
            )
            if cmd.out_of_training_range:
                self.get_logger().warn(
                    "target outside model_15200 training range "
                    f"(x in {self._cfg.safety.x_range}, |y| in {self._cfg.safety.y_abs_range}, "
                    f"z in {self._cfg.safety.z_range}) - clamped before publish."
                )
        if self._csv is not None:
            speed = float(np.linalg.norm(cmd.velocity))
            self._csv.writerow([
                f"{cmd.t:.4f}", cmd.seq, cmd.level, cmd.level_name, cmd.swing_type, cmd.frame_id,
                f"{cmd.strike_phase:.4f}", f"{cmd.time_to_strike:.4f}", f"{cmd.strike_time:.4f}",
                f"{cmd.clip_phase:.4f}",
                f"{cmd.position[0]:.4f}", f"{cmd.position[1]:.4f}", f"{cmd.position[2]:.4f}",
                f"{cmd.velocity[0]:.4f}", f"{cmd.velocity[1]:.4f}", f"{cmd.velocity[2]:.4f}",
                f"{cmd.normal[0]:.4f}", f"{cmd.normal[1]:.4f}", f"{cmd.normal[2]:.4f}",
                f"{speed:.4f}", int(cmd.valid), int(cmd.clamped), int(cmd.out_of_training_range),
                int(self._dry_run),
            ])
            self._csv_file.flush()

    def _publish_diag(self, cmd):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = "planner_imitate"
        st.hardware_id = "planner_imitate"
        st.level = DiagnosticStatus.WARN if cmd.clamped or cmd.out_of_training_range else DiagnosticStatus.OK
        st.message = f"L{cmd.level}:{cmd.level_name} {cmd.swing_type} ({'publish' if not self._dry_run else 'dry-run'})"
        st.values = [
            KeyValue(key="swing_type", value=cmd.swing_type),
            KeyValue(key="frame_id", value=cmd.frame_id),
            KeyValue(key="strike_phase", value=f"{cmd.strike_phase:.3f}"),
            KeyValue(key="time_to_strike_s", value=f"{cmd.time_to_strike:.3f}"),
            KeyValue(key="level", value=str(cmd.level)),
            KeyValue(key="clamped", value=str(bool(cmd.clamped))),
            KeyValue(key="out_of_training_range", value=str(bool(cmd.out_of_training_range))),
            KeyValue(key="dry_run", value=str(self._dry_run)),
        ]
        arr.status = [st]
        self.diag_pub.publish(arr)

    def destroy_node(self):
        if self._csv_file is not None:
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlannerImitateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
