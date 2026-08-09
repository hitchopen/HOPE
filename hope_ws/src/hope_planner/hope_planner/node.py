"""ROS 2 node wrapping the HOPE 7-DOF racket planner.

Subscribes to ball position from the mocap `/poses` stream (the avatar_pro relay
on the Avatar Pro / Chingmu VRPN path) and publishes the desired racket state on
`/racket/command` as the shared
hope_msgs/RacketCommand. Diagnostics are published at 10 Hz.

Per HOPE rules the racket pose is never measured by motion capture; the
humanoid must achieve the commanded racket state via its own forward
kinematics. See HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md.
"""

import csv
import time
from collections import Counter, deque
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

# hope_msgs is OPTIONAL (FLAT-ONLY mode): on the robot MDU only the std_msgs flat topics
# are consumed (by the C++ --planner runner), and building hope_msgs typesupport for
# aarch64 is exactly the dependency the flat wire exists to avoid. Without hope_msgs the
# node still runs and publishes /racket/command_flat + /a3/base_pose_flat; only the
# Path-B /racket/command publisher is skipped.
try:
    from hope_msgs.msg import RacketCommand
except ImportError:  # flat-only environment (e.g. the MDU)
    RacketCommand = None

from .ball_kalman_estimator import BallKalmanEstimator
from .ball_state_estimator import BallStateEstimator
from .constants import BallPhysics, PlannerConfig
from .planner import HOPEPlanner
from .spin_estimator import SpinFromQuats
from .strike_spec_planner import StrikeSpecPlanner
from .x_hit_freeze import select_stable_base_x


class HOPEPlannerNode(Node):
    """ROS 2 node for the HOPE model-based planner."""

    def __init__(self):
        super().__init__("hope_planner")

        self.declare_parameter("ball_rigid_body_name", "pingpong_ball")
        self.declare_parameter("ball_pose_index", 0)
        self.declare_parameter("x_hit", 0.0)
        # Optional legacy startup block for deployments that explicitly request
        # it. The model_21800 field runbook leaves this false: refresh state is
        # recorded as telemetry and never suppresses planner output.
        self.declare_parameter("require_x_hit_calibration", False)
        self.declare_parameter("x_hit_calibration_offset", 0.58)
        self.declare_parameter("x_hit_calibration_window_s", 0.5)
        self.declare_parameter("x_hit_calibration_max_age_s", 0.2)
        self.declare_parameter("x_hit_calibration_min_samples", 10)
        self.declare_parameter("x_hit_calibration_max_span_m", 0.01)
        # Optional local-HDU control channel used by run_rally_v10_hdu.sh. It
        # avoids spawning `ros2 service/topic` CLI participants on the
        # group-control computer merely to freeze or inspect the hit plane.
        self.declare_parameter("x_hit_calibration_request_file", "")
        self.declare_parameter("x_hit_calibration_status_file", "")
        self.declare_parameter("target_land_x", 2.055)
        self.declare_parameter("target_land_y", -0.7625)
        self.declare_parameter("delta_t_flight", 0.5)
        # Forward-prediction validity horizon. The default 2.0 s sat just above the G3
        # sweep's 1.89 s flight: early-flight fit noise pushed predicted t_strike past the
        # horizon -> valid=0 ("planner_invalid" at the runner) for the first 0.3-0.5 s of
        # every serve, delaying the pending station and eating the walk budget (0711: bh
        # windows became unreachable). Raise per venue when serves fly longer than ~1.8 s.
        self.declare_parameter("max_predict_time", 2.0)
        # For a forward-searchable incoming x/vx state, let Stage 2 continue
        # past the base horizon only if it has not found a crossing yet. This
        # changes prediction computation, not command release.
        self.declare_parameter("adaptive_predict_horizon", False)
        self.declare_parameter("max_predict_time_cap", 3.0)
        # Velocity-fit window IN SAMPLES — coupled to the venue mocap rate. The
        # venue noise MC wants >= 100 ms of window; 31 samples ≈ 103 ms at the
        # ChingMu/VRPN 300 Hz rig (the default). A faster rig must scale it or
        # the window silently shrinks below the noise floor (OptiTrack 360 Hz:
        # 31 -> ~86 ms; use round(31 * rate / 300) = 37).
        self.declare_parameter("fit_window", 31)
        self.declare_parameter("fit_window_min_span_s", 0.0)
        self.declare_parameter("fit_window_max_span_s", 0.15)
        # Telemetry denominator only. It never changes estimator readiness or
        # command publication; set to the venue's configured mocap rate.
        self.declare_parameter("expected_mocap_hz", 300.0)
        # 0 = legacy all-axis poly_order; 1/2 overrides X/Y only. This is a
        # model-selection parameter, never a release/readiness gate.
        self.declare_parameter("horizontal_poly_order", 0)
        self.declare_parameter("bounce_center_z_max", 0.20)
        # Solve rate limit; see _poses_cb. 0.0 = solve on every mocap frame (legacy).
        self.declare_parameter("solve_period_s", 0.0)
        # Optional PER-SIDE aim/flight.  Non-NaN values let a venue deliberately choose different
        # return tactics for FH/BH; they are planner design parameters, not a policy or Gate3 ground
        # truth.  FinalV3's default arena profile instead uses one opponent-half-center aim and
        # dt=0.50 for both sides.  When enabled, selection is applied to THIS prediction's Stage-2
        # intercept (no previous-plan side latch). NaN (default) keeps the shared aim.
        self.declare_parameter("target_land_y_fh", float("nan"))
        self.declare_parameter("target_land_y_bh", float("nan"))
        self.declare_parameter("delta_t_flight_fh", float("nan"))
        self.declare_parameter("delta_t_flight_bh", float("nan"))
        # Explicit forehand/backhand selection for the flat runner command.  This is a
        # geometric decision made from the predicted intercept, not from the requested
        # racket velocity: a valid per-side velocity box may cross vy=0, so sign(vy)
        # cannot be used as a swing label.  Override this midpoint after measuring the
        # two clip-specific station-relative reach offsets.
        self.declare_parameter("swing_side_split_y", -0.25)  # v12 receipt split; overlays still set it explicitly — this default only guards a missing overlay
        # Schmitt band around the reach midpoint. Without this, centimetre-scale intercept
        # noise can flip FH/BH and move the derived station by the full reach separation.
        self.declare_parameter("swing_side_hysteresis_y", 0.04)
        self.declare_parameter("drag_k", 0.1261)          # venue fit 2026-07-03 (configs/ball_physics_venue.yaml)
        self.declare_parameter("restitution_h", 0.64)     # no-spin grip equivalent (1 - a_t)
        self.declare_parameter("restitution_v", 0.9215)   # venue table e_n
        self.declare_parameter("restitution_racket", 0.654)  # paddle e const; e(u_n) exp form applied in racket_target_planner
        self.declare_parameter("use_kalman", False)          # active physics EKF; false = legacy polyfit
        self.declare_parameter("estimator_q_accel_psd", 0.1)
        self.declare_parameter("estimator_sigma_ar1_m", 0.0052)
        self.declare_parameter("estimator_chi2_gate", 16.3)
        self.declare_parameter("estimator_track_gap_s", 0.10)
        self.declare_parameter("spin_shadow_enabled", False)
        self.declare_parameter("ball_orientation_topic", "/ball/pose")
        self.declare_parameter("spin_window_s", 0.10)
        self.declare_parameter("spin_gate_rev_s", 3.0)
        self.declare_parameter("spin_max_gap_s", 0.05)
        self.declare_parameter("spin_max_rev_s", 20.0)
        self.declare_parameter("publish_strike_spec", False)  # diagnostics-only strike-spec inverse solve
        self.declare_parameter("racket_speed_budget", 10.0)   # m/s cap for the spec solve — diagnostic
                                                              # sanity bound, above venue strike speeds
                                                              # (paddle u_n fit envelope tops out 7.2 m/s)
        # --- ADAPTIVE hit plane (2026-07-04): x_hit follows the LIVE robot position ---
        # The trained policy WALKS to the strike (walk-and-strike lunge, ~0.5-0.8 m): after
        # one return the robot stands AT the old static plane, so subsequent plans land at
        # base-rel x ~ 0 and the runner's reachability gate rejects them (one swing per
        # session). With a robot pose feed the plane tracks the robot:
        #   x_hit = clamp(robot_x + x_hit_offset, x_hit_min, x_hit_max)
        # x_hit_offset = the policy's comfortable strike reach (base-rel box center ~0.67).
        # x_hit_max PROTECTS THE TABLE: the lunge marches the robot forward each swing; the
        # clamp stops the plane (and therefore the robot) at the table edge minus reach.
        # Empty robot_pose_topic -> static x_hit (legacy behavior).
        # Arena legacy: robot_pose_topic:=/P1/pose (raw marker pose).
        # V17 production instead consumes the receipt-bearing schema-2
        # base_pose_flat_input_topic so planner and native runner use the same
        # calibrated world->pelvis pose.
        # AGI sim: robot_pose_topic:=/sim/a3/pelvis_pose (sim world == table frame).
        self.declare_parameter("robot_pose_topic", "")
        self.declare_parameter("base_pose_flat_input_topic", "")
        self.declare_parameter("x_hit_offset", 0.67)
        self.declare_parameter("x_hit_min", -0.30)
        self.declare_parameter("x_hit_max", 0.30)   # table edge x=0 + racket reach margin
        # HITTER-PURE decoupling (2026-07-07): the paper's virtual hit plane is FIXED (§IV-B,
        # x = -1.37 m table frame) and the ROBOT walks to a commanded station behind it (Fig. 4);
        # the adaptive plane above inverts that causality (plane chases the robot -> the
        # documented "+x march"). For the 110-D hitter_pure deploy profile keep this FALSE:
        # robot_pose_topic keeps feeding /a3/base_pose_flat (the runner needs the live base),
        # but x_hit stays the static parameter and the runner derives the station from the
        # target (station_x = x_hit - plane_x[side], per-clip plane from the ONNX boxes;
        # v13: fh 0.65 / bh 0.50 - see x_hit_bh_delta below).
        #
        # DEFAULT FALSE (2026-07-09): fixed-plane is the SAFE default for the x-locked HITTER
        # north star. Follow-mode SILENTLY DELETES the x anchor for an x-locked policy — the
        # plane chases the robot, so the obs x-error stays ~constant, the policy never sees its
        # own +x drift, never corrects, and FALLS (RALLY_STAGE2_XLOCK.md deploy trap). A
        # forgotten flag must fail SAFE, so the default is now fixed-plane. Follow-mode is an
        # EXPLICIT legacy opt-in: the 175/177 multi-swing sim rally re-enables it in
        # hope_planner.sim.yaml, or pass -p x_hit_follow_robot:=true.
        self.declare_parameter("x_hit_follow_robot", False)
        # PER-SIDE hit planes (2026-07-13, v13 facefix): the Step-10.5 wrist re-solve moved
        # the two clips' contacts to DIFFERENT station-relative planes (fh x 0.65 / bh x 0.50,
        # receipt hitter_rally_v13_facefix_receipt.json) — a single plane puts the backhand
        # intercept ~0.15 m too far forward, the runner then derives station_x = base + 0.15
        # and its x-readiness gate (--gate-x-max 0.15) refuses every backhand engage.
        # x_hit (and follow-mode's robot_x + x_hit_offset) is the FOREHAND plane; the backhand
        # plane is x_hit + x_hit_bh_delta (v13: -0.15). Solve order: predict at the forehand
        # plane (the FARTHER one - the incoming ball crosses it first), select the side from
        # that intercept (split + hysteresis), and re-predict at the backhand plane when the
        # backhand is selected. 0.0 (default) = single-plane legacy behavior (pre-v13 clips).
        self.declare_parameter("x_hit_bh_delta", 0.0)
        # --- FLAT outputs for the AGI native C++ runner (--planner, the ONLY control path) ---
        # The C++ a3_deploy_onnx_ref_pingpong subscribes std_msgs/Float64MultiArray (it avoids
        # vendoring hope_msgs typesupport on aarch64). We MIRROR /racket/command as a flat array
        # and stream the robot base pose (from robot_pose_topic) as a second flat array so the
        # runner's external_base localization has a live base.
        self.declare_parameter("publish_flat_cmd", True)
        # Schema 2 adds producer time and revision identity. The native runner
        # logs revision stability, but model_21800 releases on the ball clock;
        # the count is not an admission condition.
        self.declare_parameter("racket_flat_schema", 1)
        # Backward-compatible in-process base relay.  Production/Gate3 can disable this and
        # launch hope_base_pose_flat_relay as a separate process so expensive ball solves can
        # never starve the fail-closed runner's localization stream.
        self.declare_parameter("publish_base_flat", True)
        self.declare_parameter("racket_flat_topic", "/racket/command_flat")
        self.declare_parameter("base_flat_topic", "/a3/base_pose_flat")
        # Independent pre-serve ball-state stream for the deterministic serve
        # controller.  It is published before the x_hit calibration and
        # expensive rally solve gates, so serving cannot deadlock on a rally-
        # only prerequisite.  The hand remains a rigid 0-DoF palm; this stream
        # provides the fresh/plausible estimate used to authorize the single
        # prequalified fixed strike.  Stale or out-of-envelope estimates are
        # intentionally fail-closed in the native controller.
        self.declare_parameter("publish_serve_ball_flat", True)
        self.declare_parameter(
            "serve_ball_flat_topic", "/serve/ball_state_flat"
        )
        # marker-cluster -> base_link offset (table frame). /P1/pose is the marker cluster; the
        # policy base is the pelvis. In sim (robot_pose_topic=/sim/a3/pelvis_pose) it is already
        # the pelvis, so [0,0,0]. Set per venue (mirrors hope_world_frame.yaml mocap_to_base_link / G8).
        self.declare_parameter("marker_to_base_xyz", [0.0, 0.0, 0.0])
        # Z offset added to ALL PUBLISHED OUTPUTS (both flats + /racket/command) converting the
        # planner's working frame into the POLICY world frame (z=0 at the FLOOR — the training
        # frame the C++ runner's gates expect: base_low 0.7, target z in [0.55,1.40]).
        # Planner INTERNALS (bounce plane z=0, net check, target_land) stay in the MOCAP frame;
        # at the arena that is the G5 calibration with z=0 at the TABLE SURFACE, so set 0.76
        # (= TableParams.height). Sim feeds are already floor-origin -> keep 0.0.
        # (Field 2026-07-07: without this, arena base z ~0.15 < base_low 0.7 -> engage never fires.)
        self.declare_parameter("policy_z_offset", 0.0)
        # Table +Y edge in the working frame (TableParams.y_max). Arena default 0.0
        # (origin at near-left corner, table at y<=0); the SIM harness centers the
        # table on the robot -> hope_planner.sim.yaml sets 0.7825.
        self.declare_parameter("table_y_max", 0.0)
        # Optional field trace. This is deliberately a file sink rather than a
        # new ROS topic so enabling diagnostics does not add DDS load on the HDU.
        self.declare_parameter("debug_csv_path", "")
        self.declare_parameter("debug_session_id", "")
        self.declare_parameter("debug_flush_rows", 64)

        self._ball_index = int(self.get_parameter("ball_pose_index").value)
        self._x_hit_offset = float(self.get_parameter("x_hit_offset").value)
        self._x_hit_min = float(self.get_parameter("x_hit_min").value)
        self._x_hit_max = float(self.get_parameter("x_hit_max").value)
        self._x_hit_follow_robot = bool(self.get_parameter("x_hit_follow_robot").value)
        self._x_hit_bh_delta = float(self.get_parameter("x_hit_bh_delta").value)
        # The static FOREHAND plane. config.x_hit is mutated by the per-side re-predict, so
        # every solve re-assigns it from here (or from the follow-mode clamp) first.
        self._x_hit_fh_static = float(self.get_parameter("x_hit").value)
        self._require_x_hit_calibration = bool(
            self.get_parameter("require_x_hit_calibration").value
        )
        # Keep "refresh seen" separate from whether a legacy deployment asks
        # for blocking behavior. With require=false, commands continue from the
        # configured fallback plane while this remains false/audit-only.
        self._x_hit_calibrated = False
        self._x_hit_calibration_offset = float(
            self.get_parameter("x_hit_calibration_offset").value
        )
        self._x_hit_calibration_window_s = float(
            self.get_parameter("x_hit_calibration_window_s").value
        )
        self._x_hit_calibration_max_age_s = float(
            self.get_parameter("x_hit_calibration_max_age_s").value
        )
        self._x_hit_calibration_min_samples = int(
            self.get_parameter("x_hit_calibration_min_samples").value
        )
        self._x_hit_calibration_max_span_m = float(
            self.get_parameter("x_hit_calibration_max_span_m").value
        )
        self._x_hit_calibration_request_file = str(
            self.get_parameter("x_hit_calibration_request_file").value
        )
        self._x_hit_calibration_status_file = str(
            self.get_parameter("x_hit_calibration_status_file").value
        )
        self._base_x_samples = deque(maxlen=4096)
        self._robot_x = None          # latest robot X (table frame); None -> static x_hit
        self._robot_y = None          # latest robot Y (table frame); side split for per-side aim
        # per-side aim/flight (NaN = disabled); consumed in _poses_cb before the solve
        self._land_y_fh = float(self.get_parameter("target_land_y_fh").value)
        self._land_y_bh = float(self.get_parameter("target_land_y_bh").value)
        self._dtf_fh = float(self.get_parameter("delta_t_flight_fh").value)
        self._dtf_bh = float(self.get_parameter("delta_t_flight_bh").value)
        self._swing_side_split_y = float(self.get_parameter("swing_side_split_y").value)
        self._swing_side_hysteresis_y = max(
            0.0, float(self.get_parameter("swing_side_hysteresis_y").value)
        )
        self._per_side_aim = not (np.isnan(self._land_y_fh) and np.isnan(self._land_y_bh)
                                  and np.isnan(self._dtf_fh) and np.isnan(self._dtf_bh))
        self._last_intercept_y = None  # last valid plan's arrival y (diagnostics)
        self._last_swing_sign = 0.0
        self._warned_no_robot_y = False  # one-shot per-side-aim fallback warning
        self._solve_period = float(self.get_parameter("solve_period_s").value)
        self._last_solve_t = None
        self._publish_flat = bool(self.get_parameter("publish_flat_cmd").value)
        self._racket_flat_schema = int(
            self.get_parameter("racket_flat_schema").value
        )
        if self._racket_flat_schema not in (1, 2):
            raise ValueError("racket_flat_schema must be 1 or 2")
        self._publish_base_flat = bool(self.get_parameter("publish_base_flat").value)
        self._publish_serve_ball_flat = bool(
            self.get_parameter("publish_serve_ball_flat").value
        )
        self._marker_to_base = np.array(
            [float(v) for v in self.get_parameter("marker_to_base_xyz").value])
        self._policy_z_offset = float(self.get_parameter("policy_z_offset").value)

        self._debug_file = None
        self._debug_csv = None
        self._debug_rows_since_flush = 0
        self._debug_flush_rows = max(1, int(self.get_parameter("debug_flush_rows").value))
        self._debug_session_id = str(self.get_parameter("debug_session_id").value)
        self._debug_command_seq = 0
        self._debug_flight_id = 0
        self._debug_last_valid_command_mono_ns = None
        # Wire identity is operational state, not debug state.  It must advance
        # even when debug_csv_path is empty (the previous counters lived inside
        # _write_debug_sample and silently stayed at zero without a CSV).
        self._wire_command_seq = 0
        self._wire_flight_id = 0
        self._wire_revision_id = 0
        self._wire_last_valid_command_mono_ns = None
        debug_csv_path = str(self.get_parameter("debug_csv_path").value).strip()
        if debug_csv_path:
            path = Path(debug_csv_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._debug_file = path.open(
                "w", newline="", encoding="utf-8", buffering=1024 * 1024
            )
            self._debug_csv = csv.writer(self._debug_file)
            self._debug_csv.writerow([
                "session_id", "flight_id", "sample_seq", "solve_seq", "command_seq",
                "receipt_wall_ns", "receipt_monotonic_ns", "ros_stamp_ns",
                "solved", "published", "reason", "x_hit_calibrated",
                "ball_x", "ball_y", "ball_z", "estimator_samples", "estimator_span_s",
                "bounce_detected",
                "estimate_valid", "est_x", "est_y", "est_z", "est_vx", "est_vy",
                "est_vz", "estimate_stamp_s", "strike_valid", "strike_ball_x",
                "strike_ball_y", "strike_ball_z", "strike_ball_vx", "strike_ball_vy",
                "strike_ball_vz", "command_available", "command_valid", "swing_sign",
                "intercept_table_x", "intercept_table_y", "intercept_table_z",
                "published_policy_x", "published_policy_y", "published_policy_z",
                "racket_target_vx", "racket_target_vy", "racket_target_vz",
                "time_to_strike_s", "strike_time_s", "predicted_bounces",
                "base_raw_x", "base_raw_y", "base_raw_z", "base_policy_x",
                "base_policy_y", "base_policy_z", "base_ros_stamp_ns",
                "base_receipt_wall_ns", "policy_z_offset", "marker_to_base_x",
                "marker_to_base_y", "marker_to_base_z", "x_hit_fh", "x_hit_active",
                "spin_valid", "spin_wx", "spin_wy", "spin_wz",
                "spin_shadow_valid", "spin_shadow_dp_x", "spin_shadow_dp_y",
                "spin_shadow_dp_z", "spin_shadow_dv_x", "spin_shadow_dv_y",
                "spin_shadow_dv_z", "spin_shadow_dt_s", "planner_reason",
                "stage1_ms", "stage2_ms", "stage3_ms", "solve_total_ms",
                "ball_age_ms", "ball_gap_ms", "ball_max_gap_ms", "base_age_ms",
                "horizontal_poly_order", "estimator_kind",
                "estimator_robust_clips", "estimator_innovation_chi2",
                "estimator_track_restarts", "estimator_last_restart_reason",
                "estimator_pos_std_x", "estimator_pos_std_y",
                "estimator_pos_std_z", "estimator_vel_std_x",
                "estimator_vel_std_y", "estimator_vel_std_z",
                "target_dy_m", "target_dz_m", "target_dt_s", "failure_counts",
            ])
            self._debug_file.flush()
            self.get_logger().info(f"planner field CSV -> {path}")

        horizontal_poly_order = int(
            self.get_parameter("horizontal_poly_order").value
        )
        if horizontal_poly_order not in (0, 1, 2):
            raise ValueError("horizontal_poly_order must be 0, 1, or 2")

        config = PlannerConfig(
            x_hit=self.get_parameter("x_hit").value,
            target_land=np.array([
                self.get_parameter("target_land_x").value,
                self.get_parameter("target_land_y").value,
                0.0,
            ]),
            delta_t_flight=self.get_parameter("delta_t_flight").value,
            C_r=self.get_parameter("restitution_racket").value,
            use_kalman=bool(self.get_parameter("use_kalman").value),
            q_accel_psd=float(
                self.get_parameter("estimator_q_accel_psd").value
            ),
            sigma_ar1_m=float(
                self.get_parameter("estimator_sigma_ar1_m").value
            ),
            chi2_gate=float(
                self.get_parameter("estimator_chi2_gate").value
            ),
            estimator_track_gap_s=float(
                self.get_parameter("estimator_track_gap_s").value
            ),
            max_predict_time=float(self.get_parameter("max_predict_time").value),
            adaptive_predict_horizon=bool(
                self.get_parameter("adaptive_predict_horizon").value
            ),
            max_predict_time_cap=float(
                self.get_parameter("max_predict_time_cap").value
            ),
            fit_window=int(self.get_parameter("fit_window").value),
            fit_window_min_span_s=float(
                self.get_parameter("fit_window_min_span_s").value
            ),
            fit_window_max_span_s=float(
                self.get_parameter("fit_window_max_span_s").value
            ),
            horizontal_poly_order=(
                None if horizontal_poly_order == 0 else horizontal_poly_order
            ),
            bounce_center_z_max=float(
                self.get_parameter("bounce_center_z_max").value
            ),
        )
        physics = BallPhysics(
            k=self.get_parameter("drag_k").value,
            C_h=self.get_parameter("restitution_h").value,
            C_v=self.get_parameter("restitution_v").value,
        )

        from .constants import TableParams
        table = TableParams(y_max=float(self.get_parameter("table_y_max").value))
        self.planner = HOPEPlanner(physics=physics, config=config, table=table)
        # Do not reuse planner.estimator: HOPEPlanner deliberately discards
        # outgoing (+x) balls and its solve path may be rate limited.  Serving
        # needs the toss state regardless of rally direction or x_hit state.
        # Horizontal serve motion is effectively linear over the compact
        # selection window.  Keep the rally planner's legacy quadratic fit,
        # but avoid a noise-amplifying quadratic endpoint derivative on the
        # independent pre-serve mailbox.
        self._serve_ball_estimator = BallStateEstimator(
            config, horizontal_poly_order=1
        )

        # Rotation-aware Stage-2 SHADOW. The legacy no-spin prediction remains
        # the sole command source until field receipts calibrate this model.
        self._spin = (
            SpinFromQuats(
                window_s=float(self.get_parameter("spin_window_s").value),
                gate_rev_s=float(self.get_parameter("spin_gate_rev_s").value),
                max_gap_s=float(self.get_parameter("spin_max_gap_s").value),
                max_rev_s=float(self.get_parameter("spin_max_rev_s").value),
            )
            if bool(self.get_parameter("spin_shadow_enabled").value)
            else None
        )
        self._spin_omega = None
        self._spin_shadow_valid = False
        self._spin_shadow_dp = np.full(3, np.nan)
        self._spin_shadow_dv = np.full(3, np.nan)
        self._spin_shadow_dt = float("nan")

        # Flag-gated strike-spec DIAGNOSTICS: inverse-solve the racket control
        # variables (face tilt, v_n, v_t) + their landing sensitivities next to
        # the existing racket command. Does NOT touch the command path. The LM
        # solve costs ~0.3 s, so it is throttled to at most 1 Hz rather than
        # running per 300 Hz mocap frame.
        self._publish_strike_spec = bool(self.get_parameter("publish_strike_spec").value)
        self._racket_speed_budget = float(self.get_parameter("racket_speed_budget").value)
        self._spec_planner = (
            StrikeSpecPlanner(physics=physics, config=config)
            if self._publish_strike_spec else None
        )
        self._last_spec = None
        self._spec_next_t = float("-inf")

        # In-process health counters. These replace HDU-side `ros2 topic
        # echo/hz` probes, which create extra DDS participants and can overload
        # the group-control computer/network.
        self._n_received = 0
        self._n_ball_present = 0
        self._n_ball_missing = 0
        self._n_valid = 0
        self._n_solves = 0
        self._n_robot_pose_received = 0
        self._expected_mocap_hz = max(
            1.0, float(self.get_parameter("expected_mocap_hz").value)
        )
        self._last_robot_pose_receipt_s = None
        self._last_ball_receipt_s = None
        self._last_ball_receipt_monotonic_ns = None
        self._last_ball_gap_s = float("nan")
        self._ball_max_gap_s = 0.0
        self._planner_reason_counts = Counter()
        self._solve_ms_recent = deque(maxlen=512)
        self._valid_solve_ms_recent = deque(maxlen=512)
        self._audit_previous_target = None
        self._health_prev_time_s = self.get_clock().now().nanoseconds * 1.0e-9
        self._health_prev_counts = (0, 0, 0, 0, 0)
        self._last_valid = False
        self._last_tts = float("nan")
        self._last_robot_raw = np.full(3, np.nan)
        self._last_robot_policy = np.full(3, np.nan)
        self._last_robot_ros_stamp_ns = 0
        self._last_robot_receipt_wall_ns = 0

        # Best-effort, depth-1 QoS for high-rate mocap topics (REP-2003 sensor style).
        mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(PoseArray, "/poses", self._poses_cb, mocap_qos)
        if self._spin is not None:
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter("ball_orientation_topic").value),
                self._ball_orientation_cb,
                mocap_qos,
            )
            self.get_logger().warning(
                "ball spin SHADOW enabled: legacy no-spin prediction remains "
                "load-bearing; rotation-aware deltas are diagnostics only"
            )
        if RacketCommand is not None:
            self.cmd_pub = self.create_publisher(RacketCommand, "/racket/command", command_qos)
        else:
            self.cmd_pub = None
            self.get_logger().warn(
                "hope_msgs not available -> FLAT-ONLY mode: /racket/command is NOT "
                "published (fine for the C++ --planner runner; Path B needs hope_msgs).")
        self.diag_pub = self.create_publisher(DiagnosticArray, "/planner/diagnostics", 1)
        self.create_service(Trigger, "~/freeze_x_hit", self._freeze_x_hit_cb)

        # Flat outputs for the AGI native C++ runner (--planner). Same RELIABLE QoS as
        # /racket/command so the AimRT ros2 subscriber (declared RELIABLE) matches.
        self.flat_cmd_pub = None
        self.flat_base_pub = None
        self.serve_ball_flat_pub = None
        if self._publish_flat:
            self.flat_cmd_pub = self.create_publisher(
                Float64MultiArray, str(self.get_parameter("racket_flat_topic").value), command_qos)
        if self._publish_base_flat:
            self.flat_base_pub = self.create_publisher(
                Float64MultiArray, str(self.get_parameter("base_flat_topic").value), command_qos)
        if self._publish_serve_ball_flat:
            self.serve_ball_flat_pub = self.create_publisher(
                Float64MultiArray,
                str(self.get_parameter("serve_ball_flat_topic").value),
                command_qos,
            )

        robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)
        if robot_pose_topic:
            def _robot_pose_cb(msg: PoseStamped) -> None:
                self._n_robot_pose_received += 1
                self._last_robot_pose_receipt_s = (
                    self.get_clock().now().nanoseconds * 1.0e-9
                )
                p = msg.pose.position
                q = msg.pose.orientation
                q_wxyz = np.array(
                    [float(q.w), float(q.x), float(q.y), float(q.z)], dtype=float
                )
                q_norm = float(np.linalg.norm(q_wxyz))
                if np.isfinite(q_norm) and q_norm > 1.0e-9:
                    q_out_wxyz = q_wxyz / q_norm
                    qw, qx, qy, qz = q_out_wxyz
                    rot = np.array([
                        [1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - qw*qz),
                         2 * (qx*qz + qw*qy)],
                        [2 * (qx*qy + qw*qz), 1 - 2 * (qx*qx + qz*qz),
                         2 * (qy*qz - qw*qx)],
                        [2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx),
                         1 - 2 * (qx*qx + qy*qy)],
                    ])
                    marker_to_base_w = rot @ self._marker_to_base
                else:
                    # A position-only relay may leave orientation unset. Preserve the documented
                    # square-start/world-aligned fallback rather than producing NaNs. Publish the
                    # same identity fallback so downstream consumers see a valid quaternion.
                    q_out_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
                    marker_to_base_w = self._marker_to_base
                bx = float(p.x) + float(marker_to_base_w[0])
                by = float(p.y) + float(marker_to_base_w[1])
                bz = float(p.z) + float(marker_to_base_w[2]) + self._policy_z_offset
                self._last_robot_raw = np.array(
                    [float(p.x), float(p.y), float(p.z)], dtype=float
                )
                self._last_robot_policy = np.array([bx, by, bz], dtype=float)
                self._last_robot_ros_stamp_ns = (
                    int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec)
                )
                self._last_robot_receipt_wall_ns = time.time_ns()
                # Side selection, adaptive-x (legacy), and the runner's base flat must use the
                # same corrected p_base. Using raw marker y here shifts the FH/BH boundary whenever
                # marker_to_base_y is nonzero and makes the explicit station inverse inconsistent.
                self._robot_x = bx
                self._robot_y = by
                if np.isfinite(bx):
                    receipt_s = self.get_clock().now().nanoseconds * 1.0e-9
                    self._base_x_samples.append((receipt_s, bx))
                # Stream the base pose to the C++ runner (external_base localization). Apply the
                # marker->base offset expressed in marker/base-local axes, rotated by the mocap
                # orientation. The runner uses position for localization; quaternion is informational
                # but is normalized here to keep the flat interface valid for other consumers.
                if self.flat_base_pub is not None:
                    m = Float64MultiArray()
                    # [schema, valid, x, y, z, qw, qx, qy, qz]
                    m.data = [1.0, 1.0, bx, by, bz,
                              float(q_out_wxyz[0]), float(q_out_wxyz[1]),
                              float(q_out_wxyz[2]), float(q_out_wxyz[3])]
                    self.flat_base_pub.publish(m)

            self.create_subscription(PoseStamped, robot_pose_topic, _robot_pose_cb, mocap_qos)
            if self._x_hit_follow_robot:
                self.get_logger().warning(
                    f"adaptive x_hit ON (FOLLOW-MODE): robot pose from '{robot_pose_topic}', "
                    f"x_hit = clamp(robot_x + {self._x_hit_offset:.2f}, "
                    f"[{self._x_hit_min:.2f}, {self._x_hit_max:.2f}]). "
                    "*** UNSAFE for x-LOCKED policies (110-D hitter_pure / RALLY_STAGE2_XLOCK): "
                    "the plane chases the robot so the policy CANNOT see its own x-drift and WILL "
                    "fall. Run fixed-plane: --params-file hope_planner.hitter_pure.yaml or "
                    "-p x_hit_follow_robot:=false. Only the legacy 175/177 multi-swing rally "
                    "wants this ON. ***")
            else:
                self.get_logger().info(
                    f"x_hit FIXED at {config.x_hit:.2f} (x-locked / hitter_pure profile, paper "
                    f"§IV-B); robot pose from '{robot_pose_topic}' feeds the base flat only")

        base_pose_flat_input_topic = str(
            self.get_parameter("base_pose_flat_input_topic").value
        )
        if base_pose_flat_input_topic:
            required_flags = (
                (1 << 0)  # tracking valid
                | (1 << 1)  # quaternion valid
                | (1 << 2)  # marker extrinsic calibrated
                | (1 << 3)  # HDU ROS source stamp
                | (1 << 5)  # venue world calibrated
            )

            def _base_flat_cb(msg: Float64MultiArray) -> None:
                values = [float(v) for v in msg.data]
                try:
                    if len(values) != 16 or values[0] != 2.0:
                        raise ValueError("expected the 16-field schema-2 base packet")
                    if values[1] != 1.0:
                        raise ValueError("base packet is explicitly invalid")
                    flags = int(values[13])
                    if flags != values[13] or flags & required_flags != required_flags:
                        raise ValueError("base packet calibration/validity flags are incomplete")
                    if values[14] <= 0.0 or values[15] <= 0.0:
                        raise ValueError("base packet receipt IDs are missing")
                    source_sec = int(values[3])
                    source_nsec = int(values[4])
                    if (
                        source_sec <= 0
                        or source_sec != values[3]
                        or source_nsec < 0
                        or source_nsec >= 1_000_000_000
                        or source_nsec != values[4]
                    ):
                        raise ValueError("base packet source timestamp is invalid")
                    position = np.asarray(values[5:8], dtype=float)
                    quaternion = np.asarray(values[8:12], dtype=float)
                    q_norm = float(np.linalg.norm(quaternion))
                    if (
                        not np.all(np.isfinite(position))
                        or not np.all(np.isfinite(quaternion))
                        or not 0.5 <= q_norm <= 1.5
                    ):
                        raise ValueError("base packet pose is non-finite or malformed")
                except ValueError as exc:
                    self._robot_x = None
                    self._robot_y = None
                    self.get_logger().warning(
                        f"authoritative base unavailable ({exc})",
                        throttle_duration_sec=2.0,
                    )
                    return

                self._n_robot_pose_received += 1
                self._last_robot_pose_receipt_s = (
                    self.get_clock().now().nanoseconds * 1.0e-9
                )
                self._last_robot_raw = np.full(3, np.nan)
                self._last_robot_policy = position
                self._last_robot_ros_stamp_ns = (
                    source_sec * 1_000_000_000 + source_nsec
                )
                self._last_robot_receipt_wall_ns = time.time_ns()
                self._robot_x = float(position[0])
                self._robot_y = float(position[1])
                receipt_s = self.get_clock().now().nanoseconds * 1.0e-9
                self._base_x_samples.append((receipt_s, self._robot_x))

            self.create_subscription(
                Float64MultiArray,
                base_pose_flat_input_topic,
                _base_flat_cb,
                command_qos,
            )
            self.get_logger().info(
                "authoritative base input ON: "
                f"'{base_pose_flat_input_topic}' schema=2; planner base output "
                f"is {'ON (legacy duplicate)' if self.flat_base_pub is not None else 'OFF'}"
            )
        if self._require_x_hit_calibration:
            calibration_action = (
                "trigger the configured local x_hit calibration request"
                if self._x_hit_calibration_request_file
                else "call ~/freeze_x_hit"
            )
            self.get_logger().warning(
                "X-HIT CALIBRATION REQUIRED: base flat remains live, but racket commands are "
                f"BLOCKED. Put the runner in PD_STAND, wait for the robot to settle, then "
                f"{calibration_action}. Do not enter MOTION before calibration succeeds.")
        if self._x_hit_bh_delta != 0.0:
            self.get_logger().info(
                f"PER-SIDE hit planes: fh {self.planner.config.x_hit:.2f} / "
                f"bh {self.planner.config.x_hit + self._x_hit_bh_delta:.2f} "
                f"(x_hit_bh_delta={self._x_hit_bh_delta:+.2f}, v13 facefix per-clip contacts)")

        # DDS diagnostics remain available at 10 Hz. The 1 Hz logger below is
        # the field health view and needs no extra subscriber process.
        self.create_timer(0.1, self._publish_diagnostics)
        self.create_timer(1.0, self._log_health)
        if self._x_hit_calibration_request_file:
            self.create_timer(0.1, self._poll_x_hit_calibration_request)

        self.get_logger().info(
            f"HOPE planner started - x_hit={config.x_hit:.2f}, "
            f"target={config.target_land}, ball_pose_index={self._ball_index}, "
            f"solve_period_s={self._solve_period:.3f}"
        )

    def _try_freeze_x_hit(self) -> tuple[bool, str]:
        """Freeze the fixed plane from a stable corrected-base X window."""
        if self._x_hit_follow_robot:
            return False, "x_hit_follow_robot=true; fixed-plane freeze is not applicable"

        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        try:
            stable = select_stable_base_x(
                self._base_x_samples,
                now_s=now_s,
                window_s=self._x_hit_calibration_window_s,
                max_age_s=self._x_hit_calibration_max_age_s,
                min_samples=self._x_hit_calibration_min_samples,
                max_span_m=self._x_hit_calibration_max_span_m,
            )
            x_hit = stable.x_m + self._x_hit_calibration_offset
            if not np.isfinite(x_hit):
                raise ValueError("derived x_hit is non-finite")
        except ValueError as exc:
            return False, f"NOT CALIBRATED: {exc}"

        param_result = self.set_parameters_atomically([
            Parameter("x_hit", Parameter.Type.DOUBLE, float(x_hit))
        ])
        if not param_result.successful:
            return False, f"NOT CALIBRATED: could not update x_hit: {param_result.reason}"

        self._x_hit_fh_static = float(x_hit)
        self.planner.config.x_hit = float(x_hit)
        self._x_hit_calibrated = True
        self._last_valid = False
        self._last_tts = float("nan")
        self._last_solve_t = None
        self.planner.estimator.reset()

        message = (
            f"CALIBRATED base_x={stable.x_m:.4f} m + "
            f"offset={self._x_hit_calibration_offset:.4f} m -> "
            f"x_hit={x_hit:.4f} m; samples={stable.samples} "
            f"span={stable.span_m:.4f} m newest_age={stable.newest_age_s:.3f} s"
        )
        return True, message

    def _freeze_x_hit_cb(self, _request: Trigger.Request, response: Trigger.Response):
        """ROS service compatibility path; field wrapper uses a local file."""
        response.success, response.message = self._try_freeze_x_hit()
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warning(response.message)
        return response

    def _poll_x_hit_calibration_request(self) -> None:
        request_path = Path(self._x_hit_calibration_request_file)
        if not request_path.exists():
            return
        try:
            request_id = request_path.read_text(encoding="utf-8").strip() or "unknown"
            request_path.unlink(missing_ok=True)
        except OSError as exc:
            self.get_logger().warning(f"could not consume x_hit request file: {exc}")
            return

        success, message = self._try_freeze_x_hit()
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)
        if not self._x_hit_calibration_status_file:
            return
        status_path = Path(self._x_hit_calibration_status_file)
        tmp_path = status_path.with_suffix(status_path.suffix + ".tmp")
        try:
            tmp_path.write_text(
                f"request={request_id}\nsuccess={1 if success else 0}\nmessage={message}\n",
                encoding="utf-8",
            )
            tmp_path.replace(status_path)
        except OSError as exc:
            self.get_logger().warning(f"could not write x_hit status file: {exc}")

    def _select_swing_sign(self, intercept_y: float, corrected_base_y: float) -> float:
        """Select FH/BH with hysteresis around the final reach-box midpoint."""
        rel_y = float(intercept_y) - float(corrected_base_y)
        lo = self._swing_side_split_y - self._swing_side_hysteresis_y
        hi = self._swing_side_split_y + self._swing_side_hysteresis_y
        if self._last_swing_sign > 0.5:
            if rel_y > hi:
                self._last_swing_sign = -1.0
        elif self._last_swing_sign < -0.5:
            if rel_y < lo:
                self._last_swing_sign = 1.0
        else:
            self._last_swing_sign = 1.0 if rel_y < self._swing_side_split_y else -1.0
        return self._last_swing_sign

    @staticmethod
    def _debug_vec3(value) -> tuple[float, float, float]:
        if value is None:
            return (float("nan"),) * 3
        try:
            return (float(value[0]), float(value[1]), float(value[2]))
        except (IndexError, TypeError, ValueError):
            return (float("nan"),) * 3

    def _next_wire_identity(self, valid: bool) -> tuple[int, int, int]:
        """Advance schema-2 identity independently of optional debug CSV."""
        self._wire_command_seq += 1
        if valid:
            now_mono_ns = time.monotonic_ns()
            if (
                self._wire_last_valid_command_mono_ns is None
                or now_mono_ns - self._wire_last_valid_command_mono_ns
                > 250_000_000
            ):
                self._wire_flight_id += 1
                self._wire_revision_id = 1
            else:
                self._wire_revision_id += 1
            self._wire_last_valid_command_mono_ns = now_mono_ns
        return (
            self._wire_command_seq,
            self._wire_flight_id,
            self._wire_revision_id,
        )

    def _publish_racket_flat(
        self,
        command=None,
        *,
        time_to_strike: float | None = None,
        receipt_monotonic_ns: int | None = None,
    ) -> bool:
        """Publish a legacy or revisioned native-runner command.

        Schema 2 uses producer wall time. Its TTS is reduced by solve latency
        measured on the monotonic clock, so ROS/mocap source time is never
        mistaken for a cross-process absolute clock.
        """
        if self.flat_cmd_pub is None:
            return False

        valid = bool(command is not None and command.valid)
        tts = float(time_to_strike) if time_to_strike is not None else 0.0
        if not np.isfinite(tts) or tts <= 0.0:
            valid = False
            tts = 0.0
        if valid and receipt_monotonic_ns is not None:
            solve_age_s = max(
                0.0, (time.monotonic_ns() - receipt_monotonic_ns) * 1.0e-9
            )
            tts = max(0.0, tts - solve_age_s)
            valid = tts > 0.0

        fm = Float64MultiArray()
        command_seq, flight_id, revision_id = self._next_wire_identity(valid)
        if self._racket_flat_schema == 1:
            fm.data = [
                1.0,
                1.0 if valid else 0.0,
                self._last_swing_sign if valid else 0.0,
                float(command.p_intercept[0]) if valid else 0.0,
                float(command.p_intercept[1]) if valid else 0.0,
                (float(command.p_intercept[2]) + self._policy_z_offset)
                if valid else 0.0,
                float(command.v_racket[0]) if valid else 0.0,
                float(command.v_racket[1]) if valid else 0.0,
                float(command.v_racket[2]) if valid else 0.0,
                tts,
                float(command.t_strike) if valid else 0.0,
                0.0,
            ]
            self.flat_cmd_pub.publish(fm)
            return valid

        producer_ns = time.time_ns()
        producer_sec, producer_nsec = divmod(producer_ns, 1_000_000_000)
        producer_wall_s = producer_ns * 1.0e-9
        estimator = self.planner.estimator
        estimator_span_s = float(estimator.sample_span_s)
        absolute_strike_wall_s = producer_wall_s + tts if valid else 0.0
        fm.data = [
            2.0,
            1.0 if valid else 0.0,
            self._last_swing_sign if valid else 0.0,
            float(command.p_intercept[0]) if valid else 0.0,
            float(command.p_intercept[1]) if valid else 0.0,
            (float(command.p_intercept[2]) + self._policy_z_offset)
            if valid else 0.0,
            float(command.v_racket[0]) if valid else 0.0,
            float(command.v_racket[1]) if valid else 0.0,
            float(command.v_racket[2]) if valid else 0.0,
            tts,
            absolute_strike_wall_s,
            0.0,
            float(producer_sec),
            float(producer_nsec),
            float(command_seq),
            float(flight_id if valid else 0),
            float(revision_id if valid else 0),
            float(estimator.sample_count) if valid else 0.0,
            estimator_span_s if valid else 0.0,
        ]
        self.flat_cmd_pub.publish(fm)
        return valid

    def _write_debug_sample(
        self,
        *,
        p_ball,
        receipt_wall_ns: int,
        receipt_monotonic_ns: int,
        ros_stamp_ns: int,
        solved: bool,
        published: bool,
        reason: str,
        command=None,
    ) -> None:
        audit = self.planner.audit
        planner_reason = (
            audit.reason
            if solved and reason in {"no_command", "command_valid", "command_invalid"}
            else reason
        )
        if solved:
            self._planner_reason_counts[planner_reason] += 1
            if np.isfinite(audit.solve_total_ms):
                self._solve_ms_recent.append(float(audit.solve_total_ms))
                if command is not None and command.valid:
                    self._valid_solve_ms_recent.append(float(audit.solve_total_ms))

        target_dy = target_dz = target_dt = float("nan")
        if command is not None and command.valid:
            current_target = np.array([
                float(command.p_intercept[1]),
                float(command.p_intercept[2]),
                float(command.t_strike),
            ])
            if self._audit_previous_target is not None:
                delta = current_target - self._audit_previous_target
                target_dy, target_dz, target_dt = (float(value) for value in delta)
            self._audit_previous_target = current_target

        ball_age_ms = (
            max(
                0.0,
                (receipt_monotonic_ns - self._last_ball_receipt_monotonic_ns)
                * 1.0e-6,
            )
            if self._last_ball_receipt_monotonic_ns is not None
            else float("nan")
        )
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        base_age_ms = (
            max(0.0, now_s - self._last_robot_pose_receipt_s) * 1.0e3
            if self._last_robot_pose_receipt_s is not None
            else float("nan")
        )
        estimator = self.planner.estimator
        horizontal_override = getattr(estimator, "horizontal_poly_order", None)
        horizontal_order = (
            -1
            if isinstance(estimator, BallKalmanEstimator)
            else (
                horizontal_override
                if horizontal_override is not None
                else self.planner.config.poly_order
            )
        )
        failure_counts = ";".join(
            f"{key}={value}"
            for key, value in sorted(self._planner_reason_counts.items())
        )
        if isinstance(estimator, BallKalmanEstimator):
            estimator_kind = "physics_ekf"
            estimator_robust_clips = estimator.rejected_count
            estimator_innovation_chi2 = estimator.last_innovation_chi2
            estimator_track_restarts = estimator.track_restart_count
            estimator_last_restart_reason = estimator.last_restart_reason
            estimator_pos_std = estimator.position_std
            estimator_vel_std = estimator.velocity_std
        else:
            estimator_kind = "polyfit"
            estimator_robust_clips = 0
            estimator_innovation_chi2 = float("nan")
            estimator_track_restarts = 0
            estimator_last_restart_reason = "none"
            estimator_pos_std = np.full(3, np.nan)
            estimator_vel_std = np.full(3, np.nan)

        if self._debug_csv is None:
            return

        estimator_span_s = float(estimator.sample_span_s)
        estimate_valid = False
        est_pos = est_vel = None
        estimate_stamp = float("nan")
        # Raw high-rate ball samples are already preserved by the relay CSV.
        # Re-fitting the estimator again for every rate-limited callback would
        # add work to the planner's hottest path, so derived state is sampled
        # only on actual solve rows (normally ~30 Hz).
        if solved and estimator.ready:
            try:
                est_pos, est_vel, estimate_stamp = estimator.estimate()
                estimate_valid = True
            except (FloatingPointError, ValueError, np.linalg.LinAlgError):
                pass

        strike = self.planner.strike_target
        strike_valid = bool(strike is not None and strike.valid)
        strike_pos = strike.p_ball if strike_valid else None
        strike_vel = strike.v_ball if strike_valid else None

        command_available = command is not None
        command_valid = bool(command_available and command.valid)

        ball = self._debug_vec3(p_ball)
        est = self._debug_vec3(est_pos)
        estv = self._debug_vec3(est_vel)
        strikep = self._debug_vec3(strike_pos)
        strikev = self._debug_vec3(strike_vel)
        intercept = self._debug_vec3(command.p_intercept if command_available else None)
        racket_vel = self._debug_vec3(command.v_racket if command_available else None)
        published_pos = (
            intercept[0], intercept[1], intercept[2] + self._policy_z_offset
        ) if command_available else (float("nan"),) * 3
        base_raw = self._debug_vec3(self._last_robot_raw)
        base_policy = self._debug_vec3(self._last_robot_policy)

        self._debug_csv.writerow([
            self._debug_session_id,
            self._wire_flight_id,
            self._n_received,
            self._n_solves,
            self._wire_command_seq,
            receipt_wall_ns,
            receipt_monotonic_ns,
            ros_stamp_ns,
            1 if solved else 0,
            1 if published else 0,
            reason,
            1 if self._x_hit_calibrated else 0,
            *ball,
            estimator.sample_count,
            estimator_span_s,
            1 if estimator.bounce_detected else 0,
            1 if estimate_valid else 0,
            *est,
            *estv,
            estimate_stamp,
            1 if strike_valid else 0,
            *strikep,
            *strikev,
            1 if command_available else 0,
            1 if command_valid else 0,
            self._last_swing_sign,
            *intercept,
            *published_pos,
            *racket_vel,
            self._last_tts,
            float(command.t_strike) if command_available else float("nan"),
            int(command.num_bounces) if command_available else -1,
            *base_raw,
            *base_policy,
            self._last_robot_ros_stamp_ns,
            self._last_robot_receipt_wall_ns,
            self._policy_z_offset,
            *self._debug_vec3(self._marker_to_base),
            self._x_hit_fh_static,
            float(self.planner.config.x_hit),
            1 if self._spin_omega is not None else 0,
            *self._debug_vec3(self._spin_omega),
            1 if self._spin_shadow_valid else 0,
            *self._debug_vec3(self._spin_shadow_dp),
            *self._debug_vec3(self._spin_shadow_dv),
            self._spin_shadow_dt,
            planner_reason,
            audit.stage1_ms if solved else float("nan"),
            audit.stage2_ms if solved else float("nan"),
            audit.stage3_ms if solved else float("nan"),
            audit.solve_total_ms if solved else float("nan"),
            ball_age_ms,
            self._last_ball_gap_s * 1.0e3,
            self._ball_max_gap_s * 1.0e3,
            base_age_ms,
            horizontal_order,
            estimator_kind,
            estimator_robust_clips,
            estimator_innovation_chi2,
            estimator_track_restarts,
            estimator_last_restart_reason,
            *self._debug_vec3(estimator_pos_std),
            *self._debug_vec3(estimator_vel_std),
            target_dy,
            target_dz,
            target_dt,
            failure_counts,
        ])
        self._debug_rows_since_flush += 1
        if self._debug_rows_since_flush >= self._debug_flush_rows:
            self._debug_file.flush()
            self._debug_rows_since_flush = 0

    def _publish_serve_ball_state(
        self, t: float, p_ball: np.ndarray | None
    ) -> None:
        if self.serve_ball_flat_pub is None:
            return

        valid = False
        sample_count = 0
        p_est = np.zeros(3, dtype=float)
        v_est = np.zeros(3, dtype=float)
        t_est = float(t) if np.isfinite(t) else 0.0
        if (
            p_ball is not None
            and np.isfinite(t)
            and np.asarray(p_ball).shape == (3,)
            and np.all(np.isfinite(p_ball))
        ):
            estimator = self._serve_ball_estimator
            # Duplicate/backward source stamps make a polynomial derivative
            # undefined.  Reset and fail closed until a fresh six-sample
            # window is rebuilt.
            if estimator.t_buffer and t <= estimator.t_buffer[-1]:
                estimator.reset()
            estimator.push(float(t), np.asarray(p_ball, dtype=float))
            sample_count = len(estimator.t_buffer)
            if estimator.ready:
                try:
                    p_fit, v_fit, fit_stamp = estimator.estimate()
                    if (
                        np.all(np.isfinite(p_fit))
                        and np.all(np.isfinite(v_fit))
                        and np.isfinite(fit_stamp)
                    ):
                        p_est = p_fit
                        v_est = v_fit
                        t_est = float(fit_stamp)
                        valid = True
                except (
                    FloatingPointError,
                    ValueError,
                    RuntimeError,
                    np.linalg.LinAlgError,
                ):
                    valid = False

        fm = Float64MultiArray()
        # [schema, valid, px,py,pz, vx,vy,vz, source_stamp_s,
        #  frame_code(0=planner/table world), estimator_sample_count].  Z is
        #  converted to the same
        # floor-origin policy frame as the existing racket/base flats.
        fm.data = [
            1.0,
            1.0 if valid else 0.0,
            float(p_est[0]),
            float(p_est[1]),
            float(p_est[2]) + self._policy_z_offset if valid else 0.0,
            float(v_est[0]),
            float(v_est[1]),
            float(v_est[2]),
            t_est,
            0.0,
            float(sample_count),
        ]
        self.serve_ball_flat_pub.publish(fm)

    def _ball_orientation_cb(self, msg: PoseStamped) -> None:
        """Feed only explicitly-valid Ball rigid-body orientations to shadow."""
        if self._spin is None:
            return
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        q = msg.pose.orientation
        self._spin_omega = self._spin.push(
            t, np.array([q.w, q.x, q.y, q.z], dtype=float)
        )

    def _update_spin_shadow(self, t: float) -> None:
        """Compare rotation-aware Stage 2 with legacy without changing commands."""
        self._spin_shadow_valid = False
        self._spin_shadow_dp.fill(np.nan)
        self._spin_shadow_dv.fill(np.nan)
        self._spin_shadow_dt = float("nan")
        if self._spin is None:
            self._spin_omega = None
            return
        self._spin_omega = self._spin.omega(t)
        legacy = self.planner.strike_target
        if (
            self._spin_omega is None
            or legacy is None
            or not legacy.valid
            or not self.planner.estimator.ready
        ):
            return
        try:
            p_est, v_est, t_est = self.planner.estimator.estimate()
            shadow = self.planner.predictor.predict(
                p_est, v_est, t_est, self._spin_omega
            )
        except (
            FloatingPointError,
            ValueError,
            RuntimeError,
            np.linalg.LinAlgError,
        ):
            return
        if not shadow.valid:
            return
        self._spin_shadow_valid = True
        self._spin_shadow_dp = shadow.p_ball - legacy.p_ball
        self._spin_shadow_dv = shadow.v_ball - legacy.v_ball
        self._spin_shadow_dt = float(shadow.t_strike - legacy.t_strike)

    def _poses_cb(self, msg: PoseArray) -> None:
        self._n_received += 1
        receipt_wall_ns = time.time_ns()
        receipt_monotonic_ns = time.monotonic_ns()
        ros_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if len(msg.poses) <= self._ball_index:
            self._n_ball_missing += 1
            self._publish_serve_ball_state(t, None)
            self._publish_racket_flat(
                None, receipt_monotonic_ns=receipt_monotonic_ns
            )
            self._write_debug_sample(
                p_ball=None,
                receipt_wall_ns=receipt_wall_ns,
                receipt_monotonic_ns=receipt_monotonic_ns,
                ros_stamp_ns=ros_stamp_ns,
                solved=False,
                published=False,
                reason="ball_pose_index_missing",
            )
            return

        # NOTE: PoseArray carries no names. Configure ball_pose_index to match
        # the ball's slot in the /poses ordering (both production relays put it first).
        pose = msg.poses[self._ball_index]
        p_ball = np.array([pose.position.x, pose.position.y, pose.position.z])
        self._n_ball_present += 1
        if self._last_ball_receipt_monotonic_ns is not None:
            self._last_ball_gap_s = max(
                0.0,
                (receipt_monotonic_ns - self._last_ball_receipt_monotonic_ns)
                * 1.0e-9,
            )
            self._ball_max_gap_s = max(
                self._ball_max_gap_s, self._last_ball_gap_s
            )
        self._last_ball_receipt_monotonic_ns = receipt_monotonic_ns
        self._last_ball_receipt_s = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )
        # Publish before x_hit refresh/rate-limit/rally-direction handling.
        self._publish_serve_ball_state(t, p_ball)
        if self._require_x_hit_calibration and not self._x_hit_calibrated:
            # Fail closed while still allowing _robot_pose_cb to feed /a3/base_pose_flat.
            # This breaks the startup cycle: PD_STAND can use the live mocap base before the
            # final fixed plane is known, but no ball can arm a planner strike prematurely.
            self._last_valid = False
            self._last_tts = float("nan")
            self._publish_racket_flat(
                None, receipt_monotonic_ns=receipt_monotonic_ns
            )
            self._write_debug_sample(
                p_ball=p_ball,
                receipt_wall_ns=receipt_wall_ns,
                receipt_monotonic_ns=receipt_monotonic_ns,
                ros_stamp_ns=ros_stamp_ns,
                solved=False,
                published=False,
                reason="x_hit_uncalibrated",
            )
            return

        # SOLVE RATE LIMIT (0711): the Stage-2+3 solve costs 10-100 ms in pure python; at a
        # 300 Hz mocap rate the single-threaded executor starves the robot-pose callback
        # that feeds /a3/base_pose_flat — the runner then walks on a FROZEN base pose
        # (EXT-BASE stale) and quietly falls. Push every sample into the estimator (the
        # polyfit wants the full 300 Hz window) but run the predict+aim solve at most every
        # solve_period_s. Demands are constant over a flight (verified offline), so ~30 Hz
        # command updates lose nothing at the 50 Hz runner. 0.0 (default) = legacy
        # every-frame solve.
        if (self._solve_period > 0.0 and self._last_solve_t is not None
                and 0.0 <= (t - self._last_solve_t) < self._solve_period):
            self.planner.estimator.push(t, p_ball)
            if self.planner.estimator.bounce_detected and self._spin is not None:
                self._spin.reset()
                self._spin_omega = None
            self._write_debug_sample(
                p_ball=p_ball,
                receipt_wall_ns=receipt_wall_ns,
                receipt_monotonic_ns=receipt_monotonic_ns,
                ros_stamp_ns=ros_stamp_ns,
                solved=False,
                published=False,
                reason="rate_limited",
            )
            return
        self._last_solve_t = t
        self._n_solves += 1

        # FOREHAND hit plane for THIS solve (config.x_hit is read per predict() call, and a
        # per-side backhand re-predict below may have left it at the bh plane — always
        # re-assign). Follow-mode tracks the live robot; fixed mode restores the static param.
        if self._robot_x is not None and self._x_hit_follow_robot:
            self.planner.config.x_hit = float(
                np.clip(self._robot_x + self._x_hit_offset, self._x_hit_min, self._x_hit_max))
        else:
            self.planner.config.x_hit = self._x_hit_fh_static

        # CRASH GUARD (field 2026-07-07): garbage measurements (e.g. a mocap feed in
        # millimetres) made the outgoing-velocity solve raise FloatingPointError and
        # KILLED the node mid-demo. A planner glitch must degrade to "no command"
        # (the runner's safe stand), never to a dead planner.
        solve_reason = "no_command"
        try:
            cmd = self.planner.update(t, p_ball)
            solve_reason = self.planner.audit.reason
        except (FloatingPointError, ValueError, np.linalg.LinAlgError) as exc:
            self.get_logger().warning(
                f"planner solve failed ({type(exc).__name__}: {exc}) - treating as no-solution; "
                "if persistent, check the mocap feed (units/units-of-metres, frame, outliers)",
                throttle_duration_sec=2.0)
            cmd = None
            solve_reason = f"{self.planner.audit.reason}:{type(exc).__name__}"
        if self.planner.estimator.bounce_detected and self._spin is not None:
            self._spin.reset()
            self._spin_omega = None

        if cmd is None:
            self._update_spin_shadow(t)
            self._last_valid = False
            self._last_tts = float("nan")
            self._publish_racket_flat(
                None, receipt_monotonic_ns=receipt_monotonic_ns
            )
            self._write_debug_sample(
                p_ball=p_ball,
                receipt_wall_ns=receipt_wall_ns,
                receipt_monotonic_ns=receipt_monotonic_ns,
                ros_stamp_ns=ros_stamp_ns,
                solved=True,
                published=False,
                reason=solve_reason,
            )
            return

        # SIDE DECISION (once per solve, from the FOREHAND-plane intercept): drives the
        # per-side hit plane, the per-side aim, and the published swing_sign. The backhand
        # re-predict below moves the intercept to the bh plane; re-selecting the side from
        # that intercept could flip it back near the split, so the decision is latched HERE
        # and never re-run in this callback. Gated on the STAGE-2 crossing only (not
        # cmd.valid): a backhand ball can fail the Stage-3 outgoing solve at the (wrong)
        # forehand plane and still be perfectly playable after the bh-plane re-predict.
        side_decided = False
        strike = self.planner.strike_target
        if strike is not None and strike.valid:
            robot_y = self._robot_y
            if robot_y is None:
                robot_y = 0.0
                if not self._warned_no_robot_y and (
                        self._per_side_aim or self._x_hit_bh_delta != 0.0):
                    self._warned_no_robot_y = True
                    self.get_logger().warning(
                        "per-side split: no robot pose received yet - using robot_y=0 "
                        "fallback for the fh/bh split (check robot_pose_topic)")
            self._select_swing_sign(float(strike.p_ball[1]), robot_y)
            side_decided = True

            # PER-SIDE HIT PLANE (2026-07-13, v13 facefix): the side was selected at the
            # forehand plane (the farther one — crossed first); when the backhand is the
            # side, re-predict Stage 2+3 at the backhand plane so the intercept, tts and
            # aim all live on the plane the bh clip actually trains (x_hit_bh_delta doc).
            if self._x_hit_bh_delta != 0.0 and self._last_swing_sign < 0.0:
                cmd = self.planner.repredict_at_plane(
                    self.planner.config.x_hit + self._x_hit_bh_delta)
                if cmd is None:
                    self._last_valid = False
                    self._last_tts = float("nan")
                    self._publish_racket_flat(
                        None, receipt_monotonic_ns=receipt_monotonic_ns
                    )
                    self._write_debug_sample(
                        p_ball=p_ball,
                        receipt_wall_ns=receipt_wall_ns,
                        receipt_monotonic_ns=receipt_monotonic_ns,
                        ros_stamp_ns=ros_stamp_ns,
                        solved=True,
                        published=False,
                        reason="backhand_repredict_failed",
                    )
                    return

            # Optional per-side tactic: rerun Stage 3 only when the configured aim changes.
            # The shared center-return FinalV3 profile does not enter this branch because
            # its per-side parameters remain NaN.
            if self._per_side_aim:
                fh = self._last_swing_sign > 0.0
                land_y = self._land_y_fh if fh else self._land_y_bh
                dtf = self._dtf_fh if fh else self._dtf_bh
                aim_changed = False
                if not np.isnan(land_y) and self.planner.config.target_land[1] != land_y:
                    self.planner.config.target_land[1] = land_y
                    aim_changed = True
                if not np.isnan(dtf) and self.planner.config.delta_t_flight != dtf:
                    self.planner.config.delta_t_flight = dtf
                    aim_changed = True
                if aim_changed:
                    cmd = self.planner.replan_latest()  # Stage 3 only; estimator/tts intact
                    if cmd is None:
                        self._last_valid = False
                        self._last_tts = float("nan")
                        self._publish_racket_flat(
                            None, receipt_monotonic_ns=receipt_monotonic_ns
                        )
                        self._write_debug_sample(
                            p_ball=p_ball,
                            receipt_wall_ns=receipt_wall_ns,
                            receipt_monotonic_ns=receipt_monotonic_ns,
                            ros_stamp_ns=ros_stamp_ns,
                            solved=True,
                            published=False,
                            reason="per_side_replan_failed",
                        )
                        return

        self._update_spin_shadow(t)
        self._last_valid = cmd.valid
        tts = self.planner.time_to_strike
        self._last_tts = tts if tts is not None else float("nan")
        if cmd.valid:
            self._n_valid += 1
            self._last_intercept_y = float(cmd.p_intercept[1])  # diagnostics only
            if not side_decided:
                # Legacy fallback (planner without a Stage-2 prediction): publish a side
                # from the command intercept. Otherwise the side was latched above.
                robot_y = self._robot_y if self._robot_y is not None else 0.0
                self._select_swing_sign(float(cmd.p_intercept[1]), robot_y)

        if self.cmd_pub is not None:
            out = RacketCommand()
            out.header = msg.header
            out.header.frame_id = "world"
            out.position.x = float(cmd.p_intercept[0])
            out.position.y = float(cmd.p_intercept[1])
            out.position.z = float(cmd.p_intercept[2]) + self._policy_z_offset
            out.velocity.x = float(cmd.v_racket[0])
            out.velocity.y = float(cmd.v_racket[1])
            out.velocity.z = float(cmd.v_racket[2])
            # RacketCommand.normal carries the unit face normal (Vector3), not an
            # orientation. IK-based controllers that need a full quaternion can call
            # quaternion_utils.normal_to_quaternion(cmd.n_racket, constrain_up=True).
            out.normal.x = float(cmd.n_racket[0])
            out.normal.y = float(cmd.n_racket[1])
            out.normal.z = float(cmd.n_racket[2])
            out.strike_time = float(cmd.t_strike)
            out.time_to_strike = float(self._last_tts)
            out.ball_velocity_outgoing.x = float(cmd.v_ball_outgoing[0])
            out.ball_velocity_outgoing.y = float(cmd.v_ball_outgoing[1])
            out.ball_velocity_outgoing.z = float(cmd.v_ball_outgoing[2])
            out.valid = bool(cmd.valid)
            out.clears_net = bool(cmd.clears_net)
            out.bypasses_net_posts = bool(cmd.bypasses_net_posts)
            out.predicted_bounces = int(cmd.num_bounces)
            self.cmd_pub.publish(out)

        # Mirror to the flat topic for the AGI C++ runner (--planner).  The Final HitterPure
        # Publish the side selected from the predicted ball intercept.  Do not infer it
        # from racket vy: the backhand box may legally straddle zero.  This avoids a
        # geometric nearest-station ambiguity on 30–35 cm lateral transitions; legacy
        # runners may continue to ignore the field.
        self._publish_racket_flat(
            cmd,
            time_to_strike=self._last_tts,
            receipt_monotonic_ns=receipt_monotonic_ns,
        )

        # Strike-spec diagnostics AFTER the command publish so the solve
        # latency never delays the command itself.
        if self._spec_planner is not None and cmd.valid and t >= self._spec_next_t:
            self._spec_next_t = t + 1.0
            strike = self.planner.strike_target
            if strike is not None and strike.valid:
                # Legacy command path is spin-blind -> omega None (zeros);
                # promote to the EKF/spin estimate when that path lands.
                self._last_spec = self._spec_planner.solve(
                    strike.p_ball, strike.v_ball, None,
                    self.planner.config.target_land[:2],
                    self._racket_speed_budget,
                )
                if self._last_spec is not None:
                    s = self._last_spec
                    self.get_logger().info(
                        "strike spec: tilt=(%.2f, %.2f) deg  v_n=%.2f  |v_t|=%.2f m/s  "
                        "land=(%.3f, %.3f)  sens: %.3f m/deg pitch, %.3f m/deg yaw, "
                        "%.3f m/(m/s) v_n, %.3f m/(m/s) v_t"
                        % (
                            s.tilt_pitch_deg, s.tilt_yaw_deg, s.v_n_signed,
                            float(np.linalg.norm(s.v_t_vec)),
                            s.landing_xy[0], s.landing_xy[1],
                            float(np.linalg.norm(s.d_landing_d_pitch)),
                            float(np.linalg.norm(s.d_landing_d_yaw)),
                            float(np.linalg.norm(s.d_landing_d_v_n)),
                            float(np.linalg.norm(s.d_landing_d_v_t)),
                        )
                    )

        self._write_debug_sample(
            p_ball=p_ball,
            receipt_wall_ns=receipt_wall_ns,
            receipt_monotonic_ns=receipt_monotonic_ns,
            ros_stamp_ns=ros_stamp_ns,
            solved=True,
            published=self.flat_cmd_pub is not None or self.cmd_pub is not None,
            reason=self.planner.audit.reason,
            command=cmd,
        )

    def _publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "hope_planner"
        status.hardware_id = "hope_planner"
        if self._require_x_hit_calibration and not self._x_hit_calibrated:
            status.level = DiagnosticStatus.WARN
            status.message = "x_hit not calibrated; racket commands blocked"
        elif not self._x_hit_calibrated:
            status.level = DiagnosticStatus.WARN
            status.message = "x_hit not refreshed; telemetry only, commands continue"
        elif self._n_received == 0:
            status.level = DiagnosticStatus.WARN
            status.message = "no /poses received yet"
        elif self._last_valid:
            status.level = DiagnosticStatus.OK
            status.message = "valid racket command"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "running; no valid strike"
        status.values = [
            KeyValue(key="poses_received", value=str(self._n_received)),
            KeyValue(key="robot_poses_received", value=str(self._n_robot_pose_received)),
            KeyValue(key="planner_solves", value=str(self._n_solves)),
            KeyValue(key="valid_commands", value=str(self._n_valid)),
            KeyValue(key="last_valid", value=str(self._last_valid)),
            KeyValue(key="time_to_strike_s", value=f"{self._last_tts:.4f}"),
            KeyValue(key="x_hit_calibrated", value=str(self._x_hit_calibrated)),
            KeyValue(key="x_hit_m", value=f"{self._x_hit_fh_static:.4f}"),
            KeyValue(key="ball_present", value=str(self._n_ball_present)),
            KeyValue(key="ball_missing", value=str(self._n_ball_missing)),
            KeyValue(
                key="expected_mocap_hz",
                value=f"{self._expected_mocap_hz:.1f}",
            ),
            KeyValue(key="planner_reason", value=self.planner.audit.reason),
            KeyValue(key="stage1_ms", value=f"{self.planner.audit.stage1_ms:.3f}"),
            KeyValue(key="stage2_ms", value=f"{self.planner.audit.stage2_ms:.3f}"),
            KeyValue(key="stage3_ms", value=f"{self.planner.audit.stage3_ms:.3f}"),
            KeyValue(
                key="solve_total_ms",
                value=f"{self.planner.audit.solve_total_ms:.3f}",
            ),
        ]
        if isinstance(self.planner.estimator, BallKalmanEstimator):
            estimator = self.planner.estimator
            status.values += [
                KeyValue(key="estimator", value="physics_ekf"),
                KeyValue(
                    key="estimator_robust_clips",
                    value=str(estimator.rejected_count),
                ),
                KeyValue(
                    key="estimator_innovation_chi2",
                    value=f"{estimator.last_innovation_chi2:.4f}",
                ),
                KeyValue(
                    key="estimator_track_restarts",
                    value=str(estimator.track_restart_count),
                ),
            ]
        else:
            status.values.append(KeyValue(key="estimator", value="polyfit"))
        if self._spin is not None:
            status.values += [
                KeyValue(
                    key="spin_shadow_spin_valid",
                    value=str(self._spin_omega is not None),
                ),
                KeyValue(
                    key="spin_shadow_omega_rad_s",
                    value=",".join(
                        f"{value:.4f}"
                        for value in self._debug_vec3(self._spin_omega)
                    ),
                ),
                KeyValue(
                    key="spin_shadow_prediction_valid",
                    value=str(self._spin_shadow_valid),
                ),
                KeyValue(
                    key="spin_shadow_position_delta_m",
                    value=",".join(
                        f"{value:.4f}" for value in self._spin_shadow_dp
                    ),
                ),
                KeyValue(
                    key="spin_shadow_velocity_delta_mps",
                    value=",".join(
                        f"{value:.4f}" for value in self._spin_shadow_dv
                    ),
                ),
                KeyValue(
                    key="spin_shadow_strike_time_delta_s",
                    value=f"{self._spin_shadow_dt:.4f}",
                ),
            ]
        if self._publish_strike_spec:
            s = self._last_spec
            if s is None:
                status.values.append(KeyValue(key="spec_valid", value="False"))
            else:
                status.values += [
                    KeyValue(key="spec_valid", value="True"),
                    KeyValue(key="spec_tilt_pitch_deg", value=f"{s.tilt_pitch_deg:.3f}"),
                    KeyValue(key="spec_tilt_yaw_deg", value=f"{s.tilt_yaw_deg:.3f}"),
                    KeyValue(key="spec_v_n_mps", value=f"{s.v_n_signed:.3f}"),
                    KeyValue(key="spec_v_t_mps", value=f"{np.linalg.norm(s.v_t_vec):.3f}"),
                    KeyValue(key="spec_landing_x_m", value=f"{s.landing_xy[0]:.3f}"),
                    KeyValue(key="spec_landing_y_m", value=f"{s.landing_xy[1]:.3f}"),
                    # Landing-sensitivity norms = the control-precision budget:
                    # how much landing error one unit of control error buys.
                    KeyValue(key="spec_dland_dpitch_m_per_deg",
                             value=f"{np.linalg.norm(s.d_landing_d_pitch):.4f}"),
                    KeyValue(key="spec_dland_dyaw_m_per_deg",
                             value=f"{np.linalg.norm(s.d_landing_d_yaw):.4f}"),
                    KeyValue(key="spec_dland_dvn_m_per_mps",
                             value=f"{np.linalg.norm(s.d_landing_d_v_n):.4f}"),
                    KeyValue(key="spec_dland_dvt_m_per_mps",
                             value=f"{np.linalg.norm(s.d_landing_d_v_t):.4f}"),
                ]
        arr.status = [status]
        self.diag_pub.publish(arr)

    def _log_health(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        dt = max(now_s - self._health_prev_time_s, 1.0e-6)
        counts = (
            self._n_received,
            self._n_robot_pose_received,
            self._n_solves,
            self._n_valid,
            self._n_ball_present,
        )
        rates = tuple((cur - prev) / dt for cur, prev in zip(counts, self._health_prev_counts))
        base_age = (
            now_s - self._last_robot_pose_receipt_s
            if self._last_robot_pose_receipt_s is not None else float("inf")
        )
        ball_age = (
            now_s - self._last_ball_receipt_s
            if self._last_ball_receipt_s is not None else float("inf")
        )
        if np.isfinite(ball_age):
            self._ball_max_gap_s = max(self._ball_max_gap_s, ball_age)
        ball_present_pct = min(
            100.0, 100.0 * rates[4] / self._expected_mocap_hz
        )
        if self._solve_ms_recent:
            solve_p50, solve_p95 = np.percentile(
                np.asarray(self._solve_ms_recent), [50.0, 95.0]
            )
        else:
            solve_p50 = solve_p95 = float("nan")
        if self._valid_solve_ms_recent:
            valid_solve_p50, valid_solve_p95 = np.percentile(
                np.asarray(self._valid_solve_ms_recent), [50.0, 95.0]
            )
        else:
            valid_solve_p50 = valid_solve_p95 = float("nan")
        failure_summary = ",".join(
            f"{key}:{value}"
            for key, value in sorted(self._planner_reason_counts.items())
            if key != "command_valid"
        ) or "none"
        self.get_logger().info(
            "HDU HEALTH poses=%.1fHz ball=%.1fHz(present=%.1f%% age=%s max_gap=%.3fs) "
            "base=%.1fHz(age=%s) solves=%.1fHz valid=%.1fHz total_valid=%d "
            "solve_ms(p50/p95)=%s/%s valid_solve_ms=%s/%s "
            "reason=%s failures=%s x_hit=%s%.4f tts=%s"
            % (
                rates[0], rates[4], ball_present_pct,
                f"{ball_age:.3f}s" if np.isfinite(ball_age) else "never",
                self._ball_max_gap_s,
                rates[1],
                f"{base_age:.3f}s" if np.isfinite(base_age) else "never",
                rates[2], rates[3], self._n_valid,
                f"{solve_p50:.2f}" if np.isfinite(solve_p50) else "none",
                f"{solve_p95:.2f}" if np.isfinite(solve_p95) else "none",
                (
                    f"{valid_solve_p50:.2f}"
                    if np.isfinite(valid_solve_p50) else "none"
                ),
                (
                    f"{valid_solve_p95:.2f}"
                    if np.isfinite(valid_solve_p95) else "none"
                ),
                self.planner.audit.reason,
                failure_summary,
                (
                    ""
                    if self._x_hit_calibrated
                    else ("BLOCKED/" if self._require_x_hit_calibration else "UNREFRESHED/")
                ),
                self._x_hit_fh_static,
                f"{self._last_tts:.3f}" if np.isfinite(self._last_tts) else "none",
            )
        )
        self._health_prev_time_s = now_s
        self._health_prev_counts = counts

    def destroy_node(self):
        if self._debug_file is not None:
            self._debug_file.flush()
            self._debug_file.close()
            self._debug_file = None
            self._debug_csv = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HOPEPlannerNode()
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
