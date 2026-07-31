from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import subprocess
from pathlib import Path


A3_ROOT = Path(__file__).resolve().parents[1]
WRAPPER = A3_ROOT / "scripts" / "run_serve_vendor_arm.sh"
BUILD = A3_ROOT / "scripts" / "build_serve_vendor_arm_pkg.sh"
COMMON = A3_ROOT / "scripts" / "serve_vendor_arm_common.sh"
RUNNER_SOURCE = (
    A3_ROOT
    / "src/a3/a3_deploy_onnx_ref/src/a3_deploy"
    / "a3_serve_vendor_arm_main.cpp"
)
MOTION = A3_ROOT / "assets/a3_runtime/serve/motions/serve_policy.csv"
MANIFEST = (
    A3_ROOT
    / "assets/a3_runtime/serve/manifests"
    / "serve_vendor_arm.json"
)


def run_sourced(script: str, *arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", "-c", f'source "$1"; shift; {script}', "_", str(WRAPPER), *arguments],
        cwd=A3_ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=os.environ.copy(),
    )


def assert_output_order(output: str, *markers: str) -> None:
    positions = [output.index(marker) for marker in markers]
    assert positions == sorted(positions), output


def test_original_stroke_timing_and_amplitude_are_preserved() -> None:
    joint_names = [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]
    velocity_limits = [
        13.613568165555769,
        13.613568165555769,
        15.707963267948966,
        15.707963267948966,
        15.707963267948966,
        12.775810124598491,
        12.775810124598491,
        13.613568165555769,
        13.613568165555769,
        15.707963267948966,
        15.707963267948966,
        15.707963267948966,
        12.775810124598491,
        12.775810124598491,
    ]
    with MOTION.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    source = [
        [math.radians(float(rows[frame][name])) for name in joint_names]
        for frame in range(1848, 3877)
    ]

    commands = source[::2]

    max_velocity_ratio = 0.0
    max_step = 0.0
    for frame in range(1, len(commands)):
        for joint in range(14):
            if joint == 4:
                continue
            step = abs(commands[frame][joint] - commands[frame - 1][joint])
            max_step = max(max_step, step)
            max_velocity_ratio = max(
                max_velocity_ratio,
                step * 100.0 / velocity_limits[joint],
            )

    runner = RUNNER_SOURCE.read_text(encoding="utf-8")
    wrapper = WRAPPER.read_text(encoding="utf-8")
    assert "constexpr double kStrokeSpeedScale = 1.00;" in runner
    assert "constexpr double kStrokeReachScale = 1.00;" in runner
    assert "ScaleStrokeTimeline" in runner
    assert "original-timing CSV stroke" in wrapper
    assert len(commands) == 1015
    assert commands[0] == source[0]
    assert commands[-1] == source[-1]
    assert math.isclose(max_step, 0.04987904428057355, abs_tol=1.0e-12)
    assert math.isclose(
        max_velocity_ratio,
        0.3175398581581125,
        abs_tol=1.0e-12,
    )
    assert max_velocity_ratio < 0.50


def test_lean_runtime_contract_is_machine_readable() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    wrapper = WRAPPER.read_text(encoding="utf-8")
    ownership = manifest["ownership"]
    runtime = manifest["runtime_contract"]
    timeline = manifest["timeline"]
    metrics = manifest["offline_metrics"]

    assert manifest["schema_version"] == 3
    assert (
        'SERVE_VENDOR_ARM_EXPECTED_MOTION_SHA256="'
        f'{manifest["source"]["sha256"]}"'
    ) in wrapper
    assert ownership["exclusive_command_publisher_at_handoff_required"] is True
    assert ownership["runtime_graph_exclusivity_rechecked"] is False
    assert runtime["initial_state_exact_name_mapping_required"] is True
    assert runtime["initial_state_freshness_required"] is True
    assert runtime["foreign_command_collision_stops_runner"] is True
    assert runtime["motion_player_auto_restore"] is True
    assert runtime["command_gap_stops_runner"] is False
    assert runtime["runtime_state_freshness_stops_runner"] is False
    assert runtime["tracking_error_stops_runner"] is False
    assert runtime["ready_quality_wait_enabled"] is False
    assert timeline["ready_settle_hold_s"] == 0.5
    assert timeline["stroke_speed_scale"] == 1.0
    assert timeline["stroke_reach_scale"] == 1.0
    assert timeline["command_stroke_frames"] == 1015
    assert timeline["trigger_to_nominal_strike_s"] == 1.06
    assert math.isclose(
        metrics["measured_command_stroke_step_rad"],
        0.04987904428057355,
        abs_tol=1.0e-15,
    )


REAL_PREFLIGHT_STUBS = r'''
log() { printf '%s\n' "$1"; }
serve_script_prepare_mdu() { log PACKAGE; }
serve_vendor_arm_prepare_runtime() { log RUNTIME; }
serve_vendor_arm_offline_validate() { log OFFLINE; }
serve_vendor_arm_require_vendor_stack() { log STACK; }
serve_vendor_arm_verify_action() { log ACTION; }
serve_vendor_arm_require_control_endpoints() { log ENDPOINTS; }
serve_vendor_arm_require_motion_player_process() { log PROCESS; }
serve_vendor_arm_require_motion_player_idle() { log IDLE; }
serve_vendor_arm_require_vendor_topics() { log TOPICS; }
serve_vendor_arm_wait_for_state_sample() { log STATE; }
serve_vendor_arm_require_real_tty() { log TTY; }
'''


def test_wrapper_uses_only_documented_process_manager_handoff() -> None:
    wrapper = WRAPPER.read_text(encoding="utf-8")
    build = BUILD.read_text(encoding="utf-8")
    common = COMMON.read_text(encoding="utf-8")
    main = wrapper[wrapper.index("serve_vendor_arm_main() {") :]

    assert WRAPPER.stat().st_mode & 0o111
    assert "http://127.0.0.1:50080/json" in wrapper
    assert "/stop_app" not in wrapper
    assert "stop_app" in wrapper
    assert "start_app" in wrapper
    assert """--data '{"app_name":"motion_player"}'""" in wrapper
    assert "aimdk.protocol.EmAppService" not in wrapper
    assert "DisableMotionPlayer" not in wrapper
    assert "EnableMotionPlayer" not in wrapper
    assert "SERVE_VENDOR_ARM_RESTORE_REQUIRED=1" in wrapper
    assert "serve_vendor_arm_stop_runner" in wrapper
    assert '"${command[@]}" </dev/tty &' in wrapper
    assert "[[ -r /dev/tty && -w /dev/tty ]]" in wrapper
    assert "--handoff-ready-file" in wrapper
    assert "serve_vendor_arm_wait_runner_ready" in wrapper
    assert "serve_vendor_arm_release_runner" in wrapper
    assert "SIGUSR1" in wrapper
    assert '[[ "${marker}" == "RUNNING" ]]' in wrapper
    assert "runner takeover PASS" in wrapper
    assert "timeout 15s ros2 topic echo" in wrapper
    assert "within fifteen seconds" in wrapper
    assert "rm -rf" not in wrapper
    assert (
        "serve_vendor_arm_stop_runner\n"
        "  if ! serve_vendor_arm_restore"
        in wrapper
    )
    assert main.count("serve_vendor_arm_wait_for_state_sample") == 1
    real_path = main[main.index("serve_vendor_arm_prepare_runtime") :]
    for redundant_real_check in (
        "serve_vendor_arm_offline_validate",
        "serve_vendor_arm_require_control_endpoints",
        "serve_vendor_arm_require_motion_player_process",
        "serve_vendor_arm_require_vendor_topics",
        "serve_vendor_arm_wait_for_state_sample",
    ):
        assert redundant_real_check not in real_path
    assert_output_order(
        real_path,
        "serve_vendor_arm_prepare_runtime",
        'serve_vendor_arm_launch_runner "${mode}"',
        "serve_vendor_arm_wait_runner_ready",
        "serve_vendor_arm_stop_motion_player_app",
        "serve_vendor_arm_release_runner",
        "serve_vendor_arm_wait_runner_started",
    )
    assert "run_serve_vendor_arm.sh" in build
    assert "run_serve_vendor_arm_real.sh" not in build
    assert "run_serve_vendor_arm_shadow.sh" not in build
    assert "a3_serve_script_runner" not in build
    assert "a3_serve_vendor_arm_runner" in common


def test_package_manifest_rejects_tampering_and_untracked_files(
    tmp_path: Path,
) -> None:
    package = tmp_path / "a3_serve_vendor_arm"
    required = {
        "a3_serve_vendor_arm_runner": b"runner\n",
        "run_serve_vendor_arm.sh": b"wrapper\n",
        "serve_vendor_arm_common.sh": b"common\n",
        "motions/serve_policy.csv": b"motion\n",
        "config/serve_vendor_arm_manifest.json": b"{}\n",
        "config/serve_vendor_arm_build.env": b"build\n",
    }
    for relative, payload in required.items():
        path = package / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)

    package_manifest = package / "config/serve_vendor_arm_package.sha256"
    package_manifest.write_text(
        "".join(
            f"{hashlib.sha256(payload).hexdigest()}  {relative}\n"
            for relative, payload in sorted(required.items())
        ),
        encoding="utf-8",
    )
    probe = r'''
SERVE_SCRIPT_DEPLOY_DIR="$1"
SERVE_VENDOR_ARM_PACKAGE_MANIFEST="$1/config/serve_vendor_arm_package.sha256"
serve_vendor_arm_verify_package_manifest
'''

    accepted = subprocess.run(
        ["bash", "-c", 'source "$1"; shift; ' + probe, "_", str(COMMON), str(package)],
        cwd=A3_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert accepted.returncode == 0, accepted.stderr

    (package / "a3_serve_vendor_arm_runner").write_text(
        "tampered\n",
        encoding="utf-8",
    )
    tampered = subprocess.run(
        ["bash", "-c", 'source "$1"; shift; ' + probe, "_", str(COMMON), str(package)],
        cwd=A3_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert tampered.returncode != 0
    assert "package SHA mismatch" in tampered.stderr

    (package / "a3_serve_vendor_arm_runner").write_bytes(
        required["a3_serve_vendor_arm_runner"]
    )
    (package / "unexpected.txt").write_text("extra\n", encoding="utf-8")
    untracked = subprocess.run(
        ["bash", "-c", 'source "$1"; shift; ' + probe, "_", str(COMMON), str(package)],
        cwd=A3_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert untracked.returncode != 0
    assert "unmanifested package file" in untracked.stderr


def test_process_manager_response_parser_rejects_explicit_failure(
    tmp_path: Path,
) -> None:
    success = tmp_path / "success.json"
    failure = tmp_path / "failure.json"
    invalid = tmp_path / "invalid.json"
    success.write_text(json.dumps({"success": True}), encoding="utf-8")
    failure.write_text(json.dumps({"success": False}), encoding="utf-8")
    invalid.write_text("not-json\n", encoding="utf-8")

    accepted = run_sourced(
        'serve_vendor_arm_parse_process_manager_success "$1"',
        str(success),
    )
    rejected = run_sourced(
        'serve_vendor_arm_parse_process_manager_success "$1"',
        str(failure),
    )
    malformed = run_sourced(
        'serve_vendor_arm_parse_process_manager_success "$1"',
        str(invalid),
    )

    assert accepted.returncode == 0, accepted.stderr
    assert rejected.returncode != 0
    assert "reported failure" in rejected.stderr
    assert malformed.returncode != 0
    assert "invalid process-manager JSON" in malformed.stderr


def test_motion_status_parser_requires_idle_or_stop(tmp_path: Path) -> None:
    idle = tmp_path / "idle.json"
    bad = tmp_path / "bad.json"
    idle.write_text(
        json.dumps(
            {
                "header": {"code": "0"},
                "status": "MotionCommandStatus_IDLE",
            }
        ),
        encoding="utf-8",
    )
    bad.write_text(
        json.dumps(
            {
                "header": {"code": "7"},
                "status": "MotionCommandStatus_IDLE",
            }
        ),
        encoding="utf-8",
    )

    accepted = run_sourced(
        'serve_vendor_arm_parse_motion_status "$1"',
        str(idle),
    )
    rejected = run_sourced(
        'serve_vendor_arm_parse_motion_status "$1"',
        str(bad),
    )

    assert accepted.returncode == 0, accepted.stderr
    assert accepted.stdout.strip() == "MotionCommandStatus_IDLE"
    assert rejected.returncode != 0
    assert "header.code must be 0" in rejected.stderr


def test_default_preflight_never_mutates_motion_player() -> None:
    probe = r'''
log() { printf '%s\n' "$1"; }
serve_script_prepare_mdu() { log package; }
serve_vendor_arm_offline_validate() { log offline; }
serve_vendor_arm_require_vendor_stack() { log stack; }
serve_vendor_arm_verify_action() { log action; }
serve_vendor_arm_require_control_endpoints() { log endpoints; }
serve_vendor_arm_require_motion_player_process() { log process; }
serve_vendor_arm_require_motion_player_idle() { log idle; }
serve_vendor_arm_require_vendor_topics() { log topics; }
serve_vendor_arm_wait_for_state_sample() { log state; }
serve_vendor_arm_stop_motion_player_app() { log STOP_APP_CALLED; }
serve_vendor_arm_start_motion_player_app() { log START_APP_CALLED; }
serve_vendor_arm_main
'''
    result = run_sourced(probe)

    assert result.returncode == 0, result.stderr
    expected = [
        "package",
        "offline",
        "stack",
        "action",
        "endpoints",
        "process",
        "idle",
        "topics",
        "state",
    ]
    positions = [result.stdout.index(item) for item in expected]
    assert positions == sorted(positions)
    assert "PREFLIGHT PASS" in result.stdout
    assert "STOP_APP_CALLED" not in result.stdout
    assert "START_APP_CALLED" not in result.stdout


def test_real_handoff_prearms_before_stop_and_restores_after_completion() -> None:
    probe = REAL_PREFLIGHT_STUBS + r'''
serve_vendor_arm_stop_motion_player_app() {
  log STOP_APP
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=1
}
serve_vendor_arm_launch_runner() {
  log "LAUNCH:$1"
  sleep 0.05 &
  SERVE_VENDOR_ARM_RUNNER_PID=$!
}
serve_vendor_arm_wait_runner_ready() { log PREARMED; }
serve_vendor_arm_release_runner() { log RELEASE_SIGUSR1; }
serve_vendor_arm_wait_runner_started() { log PUBLISHER_UP; }
serve_vendor_arm_start_motion_player_app() {
  log START_APP
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=0
}
serve_vendor_arm_main --hold-only --confirm-real-commands
'''
    result = run_sourced(probe)

    assert result.returncode == 0, result.stderr
    for removed_real_probe in (
        "PACKAGE",
        "OFFLINE",
        "ENDPOINTS",
        "PROCESS",
        "TOPICS",
        "STATE",
        "TTY",
    ):
        assert removed_real_probe not in result.stdout
    assert_output_order(
        result.stdout,
        "RUNTIME",
        "STACK",
        "ACTION",
        "IDLE",
        "LAUNCH:hold-only",
        "PREARMED",
        "STOP_APP",
        "RELEASE_SIGUSR1",
        "PUBLISHER_UP",
        "START_APP",
    )
    assert "custom arm takeover PASS" in result.stdout


def test_real_runtime_hot_path_has_no_observation_stop_gates() -> None:
    runner = RUNNER_SOURCE.read_text(encoding="utf-8")
    hot_path = runner[
        runner.index("const auto publish_tick =")
        : runner.index('if (args.mode == "hold-only")')
    ]

    for removed_gate in (
        "kMaxCommandGap",
        "command publish gap exceeded",
        "CheckRuntimeSafety",
        "tracking watchdog",
        "kRuntimeStateFreshness",
        "CurrentMaxTrackingError",
        "READY quality observation",
        "kForeignCommandProbe",
        "ownership_thread_",
    ):
        assert removed_gate not in runner
    for forbidden_hot_path_call in (
        "LatestState",
        "ExistingCommandPublisherCount",
        "CheckRuntimeSafety",
        "CurrentMaxTrackingError",
        "kMaxCommandGap",
    ):
        assert forbidden_hot_path_call not in hot_path
    assert "if (g_stop) return false;" in hot_path
    assert "if (!CheckRuntimeMonitoring(io, error)) return false;" in hot_path
    assert "io.Publish(command);" in hot_path
    assert "received a foreign arm command after runner publication began" in runner
    assert "constexpr std::size_t kReadySettleTicks = 50;" in runner
    assert "constexpr std::size_t kTriggerHoldTicks = 100;" in runner


def test_retained_handoff_contract_order_is_stable() -> None:
    runner = RUNNER_SOURCE.read_text(encoding="utf-8")
    real = runner[
        runner.index("int RunReal(")
        : runner.index("\n}  // namespace")
    ]

    assert_output_order(
        real,
        "WaitForFreshState(io, entry, error)",
        "command_publishers != 1",
        "publishers == 0",
        "cached_state_age > kHandoffStateFreshness",
        'InHardLimits(entry.q, error, "cached handoff arm state")',
        "state_reacquired = state_reacquired || resumed_now",
        'WriteHandoffMarker(args.handoff_ready_file, "RUNNING", error)',
    )


def test_prearm_failure_never_stops_motion_player_or_releases_runner() -> None:
    probe = REAL_PREFLIGHT_STUBS + r'''
serve_vendor_arm_launch_runner() {
  log LAUNCH
  sleep 2 &
  SERVE_VENDOR_ARM_RUNNER_PID=$!
}
serve_vendor_arm_wait_runner_ready() {
  log PREARM_FAILED
  return 1
}
serve_vendor_arm_stop_motion_player_app() { log STOP_APP_CALLED; }
serve_vendor_arm_release_runner() { log RELEASE_CALLED; }
serve_vendor_arm_start_motion_player_app() { log START_APP_CALLED; }
serve_vendor_arm_main --hold-only --confirm-real-commands
'''
    result = run_sourced(probe)

    assert result.returncode != 0
    assert_output_order(result.stdout, "LAUNCH", "PREARM_FAILED")
    assert "STOP_APP_CALLED" not in result.stdout
    assert "RELEASE_CALLED" not in result.stdout
    assert "START_APP_CALLED" not in result.stdout


def test_stop_failure_restores_without_releasing_prearmed_runner() -> None:
    probe = REAL_PREFLIGHT_STUBS + r'''
serve_vendor_arm_launch_runner() {
  log LAUNCH
  sleep 2 &
  SERVE_VENDOR_ARM_RUNNER_PID=$!
}
serve_vendor_arm_wait_runner_ready() { log PREARMED; }
serve_vendor_arm_stop_motion_player_app() {
  log STOP_FAILED
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=1
  return 1
}
serve_vendor_arm_release_runner() { log RELEASE_CALLED; }
serve_vendor_arm_stop_runner() {
  log STOP_RUNNER
  if [[ -n "${SERVE_VENDOR_ARM_RUNNER_PID}" ]]; then
    kill -TERM "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || true
    wait "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || true
    SERVE_VENDOR_ARM_RUNNER_PID=""
  fi
}
serve_vendor_arm_start_motion_player_app() {
  log START_APP
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=0
}
serve_vendor_arm_main --hold-only --confirm-real-commands
'''
    result = run_sourced(probe)

    assert result.returncode != 0
    assert_output_order(
        result.stdout,
        "LAUNCH",
        "PREARMED",
        "STOP_FAILED",
        "STOP_RUNNER",
        "START_APP",
    )
    assert "RELEASE_CALLED" not in result.stdout


def test_release_failure_stops_runner_before_restoring_motion_player() -> None:
    probe = REAL_PREFLIGHT_STUBS + r'''
serve_vendor_arm_launch_runner() {
  log LAUNCH
  sleep 2 &
  SERVE_VENDOR_ARM_RUNNER_PID=$!
}
serve_vendor_arm_wait_runner_ready() { log PREARMED; }
serve_vendor_arm_stop_motion_player_app() {
  log STOP_APP
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=1
}
serve_vendor_arm_release_runner() {
  log RELEASE_FAILED
  return 1
}
serve_vendor_arm_wait_runner_started() { log PUBLISHER_WAIT_CALLED; }
serve_vendor_arm_stop_runner() {
  log STOP_RUNNER
  if [[ -n "${SERVE_VENDOR_ARM_RUNNER_PID}" ]]; then
    kill -TERM "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || true
    wait "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || true
    SERVE_VENDOR_ARM_RUNNER_PID=""
  fi
}
serve_vendor_arm_start_motion_player_app() {
  log START_APP
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=0
}
serve_vendor_arm_main --hold-only --confirm-real-commands
'''
    result = run_sourced(probe)

    assert result.returncode != 0
    assert_output_order(
        result.stdout,
        "LAUNCH",
        "PREARMED",
        "STOP_APP",
        "RELEASE_FAILED",
        "STOP_RUNNER",
        "START_APP",
    )
    assert "PUBLISHER_WAIT_CALLED" not in result.stdout


def test_publisher_failure_stops_runner_before_restoring_motion_player() -> None:
    probe = REAL_PREFLIGHT_STUBS + r'''
serve_vendor_arm_launch_runner() {
  log LAUNCH
  sleep 2 &
  SERVE_VENDOR_ARM_RUNNER_PID=$!
}
serve_vendor_arm_wait_runner_ready() { log PREARMED; }
serve_vendor_arm_stop_motion_player_app() {
  log STOP_APP
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=1
}
serve_vendor_arm_release_runner() { log RELEASE_SIGUSR1; }
serve_vendor_arm_wait_runner_started() {
  log PUBLISHER_FAILED
  return 1
}
serve_vendor_arm_stop_runner() {
  log STOP_RUNNER
  if [[ -n "${SERVE_VENDOR_ARM_RUNNER_PID}" ]]; then
    kill -TERM "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || true
    wait "${SERVE_VENDOR_ARM_RUNNER_PID}" 2>/dev/null || true
    SERVE_VENDOR_ARM_RUNNER_PID=""
  fi
}
serve_vendor_arm_start_motion_player_app() {
  log START_APP
  SERVE_VENDOR_ARM_RESTORE_REQUIRED=0
}
serve_vendor_arm_main --hold-only --confirm-real-commands
'''
    result = run_sourced(probe)

    assert result.returncode != 0
    assert_output_order(
        result.stdout,
        "LAUNCH",
        "PREARMED",
        "STOP_APP",
        "RELEASE_SIGUSR1",
        "PUBLISHER_FAILED",
        "STOP_RUNNER",
        "START_APP",
    )


def test_real_mode_without_literal_confirmation_stops_before_preflight() -> None:
    probe = r'''
serve_script_prepare_mdu() { printf '%s\n' PACKAGE_CALLED; }
serve_vendor_arm_stop_motion_player_app() { printf '%s\n' STOP_APP_CALLED; }
serve_vendor_arm_main --prepare-only
'''
    result = run_sourced(probe)

    assert result.returncode != 0
    assert "requires the literal --confirm-real-commands" in result.stderr
    assert "PACKAGE_CALLED" not in result.stdout
    assert "STOP_APP_CALLED" not in result.stdout
