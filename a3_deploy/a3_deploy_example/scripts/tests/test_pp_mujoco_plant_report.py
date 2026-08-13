import csv
import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "pp_mujoco_plant_report.py"
SPEC = importlib.util.spec_from_file_location("pp_mujoco_plant_report", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write(path, fields, rows):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_build_report_reads_synchronized_plant_and_runner(tmp_path):
    plant = tmp_path / "plant.csv"
    trace = tmp_path / "trace.csv"
    plant_fields = [
        "sim_time", "wall_time_ns", "reset_seq", "base_z", "base_qw", "base_qx",
        "base_qy", "base_qz", "base_vx", "base_vy", "racket_vx", "racket_vy",
        "racket_vz", "left_foot_vx", "left_foot_vy", "right_foot_vx",
        "right_foot_vy", "ctrl_sat_count", "max_ctrl_ratio",
        "left_foot_normal_force", "right_foot_normal_force", "ctrl_ratio_knee",
    ]
    _write(plant, plant_fields, [
        dict.fromkeys(plant_fields, 0) | {"sim_time": 0.0, "wall_time_ns": 900_000_000,
            "base_z": 0.1, "base_qx": 1, "left_foot_normal_force": 0,
            "right_foot_normal_force": 0, "max_ctrl_ratio": 0.0,
            "ctrl_ratio_knee": 0.0},
        dict.fromkeys(plant_fields, 0) | {"sim_time": 0.1, "wall_time_ns": 1_020_000_000,
            "base_z": 1.05, "base_qw": 1, "left_foot_normal_force": 20,
            "right_foot_normal_force": 20, "max_ctrl_ratio": 0.5,
            "ctrl_ratio_knee": 0.5},
        dict.fromkeys(plant_fields, 0) | {"sim_time": 0.2, "wall_time_ns": 1_080_000_000,
            "base_z": 1.00, "base_qw": 1, "left_foot_normal_force": 20,
            "right_foot_normal_force": 0, "max_ctrl_ratio": 1.2,
            "ctrl_sat_count": 1, "ctrl_ratio_knee": 1.2},
    ])
    runner_fields = ["wall_time_ns", "mode", "level", "des_knee", "q_knee", "qd_knee"]
    _write(trace, runner_fields, [
        {"wall_time_ns": 1_010_000_000, "mode": 3, "level": 0,
         "des_knee": 1, "q_knee": 0.8, "qd_knee": 2},
        {"wall_time_ns": 1_090_000_000, "mode": 3, "level": 1,
         "des_knee": 1, "q_knee": 0.9, "qd_knee": 3},
    ])
    report = MODULE.build_report(plant, trace)
    assert report["wall_time_overlap_s"] == 0.07
    assert report["plant"]["base_z_min_m"] == 0.1
    assert report["plant_motion"]["base_z_min_m"] == 1.0
    assert report["plant_motion"]["ctrl_saturation_row_fraction"] == 0.5
    assert report["plant"]["top_ctrl_ratio_by_joint"]["knee"] == 1.2
    assert report["runner"]["motion_rows"] == 2
    assert report["runner"]["top_qd_peak_by_joint_radps"]["knee"] == 3.0


def _write_idle_case(tmp_path, *, entry_des=0.01, chatter=False):
    plant = tmp_path / "plant_idle.csv"
    trace = tmp_path / "trace_idle.csv"
    runner_fields = ["wall_time_ns", "mode", "level", "des_joint", "q_joint", "qd_joint"]
    runner_rows = []
    for index in range(5):
        runner_rows.append({
            "wall_time_ns": index * 100_000_000,
            "mode": 1,
            "level": 0,
            "des_joint": 0.0,
            "q_joint": 0.0,
            "qd_joint": 0.0,
        })
    for index in range(31):
        desired = ((0.02 if index % 2 else -0.02) if chatter else entry_des)
        runner_rows.append({
            "wall_time_ns": (index + 5) * 100_000_000,
            "mode": 3,
            "level": 0,
            "des_joint": desired,
            "q_joint": 0.0 if chatter else desired,
            "qd_joint": 0.8 if chatter else 0.0,
        })
    runner_rows.append({
        "wall_time_ns": 3_600_000_000,
        "mode": 3,
        "level": 1,
        "des_joint": 0.0,
        "q_joint": 0.0,
        "qd_joint": 0.0,
    })
    _write(trace, runner_fields, runner_rows)

    plant_fields = [
        "wall_time_ns", "ctrl_sat_count", "ctrl_joint", "ctrl_ratio_joint",
    ]
    plant_rows = []
    for index in range(181):
        sign = -1.0 if index % 2 else 1.0
        plant_rows.append({
            "wall_time_ns": 500_000_000 + index * 20_000_000,
            "ctrl_sat_count": 0,
            "ctrl_joint": sign if chatter else 1.0,
            "ctrl_ratio_joint": 0.15 if chatter else 0.05,
        })
    _write(plant, plant_fields, plant_rows)
    return plant, trace


def _short_idle_limits():
    return MODULE.SmoothnessLimits(min_idle_s=2.0, trim_s=0.5)


def test_idle_smoothness_passes_quiet_policy_native_hold(tmp_path):
    plant, trace = _write_idle_case(tmp_path)
    report = MODULE.evaluate_idle_smoothness(plant, trace, _short_idle_limits())

    assert report["pass"] is True
    assert report["idle_window"]["duration_s"] == 3.0
    assert report["m_entry"]["worst"]["qdes_step_peak_rad"]["value"] == 0.01
    assert report["plant_effort_steady"]["ctrl_saturation_row_fraction"] == 0.0


def test_idle_smoothness_ends_at_timestamped_first_fake_serve(tmp_path):
    plant, trace = _write_idle_case(tmp_path)
    ball_log = tmp_path / "ball.log"
    ball_log.write_text(
        "[INFO] [2.000000000] [fake_ball_publisher]: serve 1: p=[3.7, 0.1, 0.5]\n")

    report = MODULE.evaluate_idle_smoothness(
        plant, trace, MODULE.SmoothnessLimits(min_idle_s=1.0, trim_s=0.5), ball_log)

    assert report["pass"] is True
    assert report["definition"].endswith("before the first fake-ball serve")
    assert report["idle_window"]["first_fake_serve_wall_time_ns"] == 2_000_000_000
    assert report["idle_window"]["wall_time_last_ns"] == 1_900_000_000


def test_idle_smoothness_rejects_m_entry_jump(tmp_path):
    plant, trace = _write_idle_case(tmp_path, entry_des=0.10)
    report = MODULE.evaluate_idle_smoothness(plant, trace, _short_idle_limits())

    assert report["pass"] is False
    assert any("m-entry q_des step peak" in failure for failure in report["failures"])


def test_idle_smoothness_rejects_command_and_effort_chatter(tmp_path):
    plant, trace = _write_idle_case(tmp_path, chatter=True)
    report = MODULE.evaluate_idle_smoothness(plant, trace, _short_idle_limits())

    assert report["pass"] is False
    assert any("q_des step RMS" in failure for failure in report["failures"])
    assert any("q_des reversals" in failure for failure in report["failures"])
    assert any("actuator-effort step RMS" in failure for failure in report["failures"])
    assert any("actuator-effort step peak" in failure for failure in report["failures"])


def test_model21800_wrapper_requires_native_no_ball_smoothness_gate():
    scripts = SCRIPT.parent
    wrapper = (scripts / "pp_gate3_hitter_pingpong.sh").read_text()
    assert "export PP_REQUIRE_PLANT_TRACE=1" in wrapper
    assert "export PP_REQUIRE_IDLE_SMOOTHNESS=1" in wrapper
    assert 'export PP_MOTION_IDLE_S="${PP_MOTION_IDLE_S:-20.0}"' in wrapper
    assert 'export PP_MIN_MOTION_IDLE_S="${PP_MIN_MOTION_IDLE_S:-15.0}"' in wrapper
    assert "--gate3-qdes-audit-only" in wrapper
    assert "cmake-build-model21800-gate3/install" in wrapper
    assert "instrumented MuJoCo install missing" in wrapper

    engine = (scripts / "pp_gate3_rally.sh").read_text()
    assert "PP_MOTION_IDLE_S='$MOTION_IDLE_S'" in engine
    launcher = (scripts / "pp_gate3_ball_launcher.py").read_text()
    assert "self._spin_until(lambda: False, self._args.motion_idle_s)" in launcher
    assert "--ball-log /tmp/pp_ball.log" in engine
    assert "--require-idle-smoothness" in engine
    assert '[ "${PP_REQUIRE_IDLE_SMOOTHNESS:-0}" = "1" ]' in engine
    assert "PP_GATE3_PREFLIGHT_ONLY" in engine
