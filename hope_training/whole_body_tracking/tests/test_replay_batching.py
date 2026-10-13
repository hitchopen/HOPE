"""Execute production ring writes without Isaac; verify provenance and overwrite semantics."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace, MethodType

import pytest

torch = pytest.importorskip("torch")
SOURCE = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py"
ACTION_FIELDS = (
    "raw", "applied_raw", "unclamped_qdes", "processed_qdes", "commanded_qdes",
    "executed_qdes", "previous_executed_qdes", "qdes_delta", "previous_qdes_delta",
    "qdes_second_difference", "decoder_offset", "decoder_scale", "delay_queue", "delay_steps",
)


def production_methods(source=SOURCE):
    owner = next(n for n in ast.parse(Path(source).read_text()).body
                 if isinstance(n, ast.ClassDef) and n.name == "MotionCommand")
    wanted = {"_capture_post_swing_states", "_ensure_v17_replay_storage", "_store_v17_target_state",
              "_v17_replay_bucket_total", "_v17_replay_bucket_label", "_v17_replay_host_counts",
              "_record_v17_replay_host_capture", "_v17_replay_ready", "_clear_post_swing_replay_bank",
              "_update_v17_replay_metrics", "_v17_eligible_replay_buckets", "_ability_gated_post_probability"}
    namespace = {"torch": torch, "math": math}
    for node in owner.body:
        if isinstance(node, ast.FunctionDef) and node.name in wanted:
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
    return {name: namespace[name] for name in wanted if name in namespace}


def fixture(n=36, capacity=3, device="cpu", source=SOURCE, target_fields=4):
    m = SimpleNamespace(num_envs=n, device=device, _post_swing_replay_contract="markov_side_phase_severity_v3",
        cfg=SimpleNamespace(post_swing_capture_phase_bins=4, post_swing_capture_severity_bins=3,
                            post_swing_buffer_size=24 * capacity, post_swing_min_fill_per_bucket=1,
                            post_swing_min_fill=3, post_swing_action_name="joint_pos", rally_timing_enabled=True),
        _v17_replay_root=None, _v17_replay_bucket_count=None, _v17_replay_bucket_count_host=None,
        _v17_replay_total_count=0, _v17_replay_ready_latched=False, _post_swing_ability_unlocked=True,
        _v17_replay_target_state={}, _v17_replay_local_latest_slot=None,
        motion=SimpleNamespace(num_segments=2, seg_start=torch.tensor([0,100], device=device),
                               seg_len=torch.tensor([100,100], device=device)))
    ids = torch.arange(n, device=device)
    m.clip_id = ids % 2
    m.time_steps = m.clip_id * 100 + 25
    m.hold_counter = ids % 4
    m.rally_wrap_step = m.clip_id * 100 + 60
    m.rally_shortened = ids % 2 == 0
    m.metrics = {"rally_scheduled_pre_s": ids.float() / 10,
                 "rally_scheduled_post_s": ids.float() / 20}
    def rows(width):
        return (ids[:,None] * 100 + torch.arange(width, device=device)).float()
    data = SimpleNamespace(root_state_w=rows(13), joint_pos=rows(31), joint_vel=rows(31)+1,
                           joint_stiffness=rows(31)+2, joint_damping=rows(31)+3)
    m.robot = SimpleNamespace(data=data)
    action_values = {name: rows(31)+i for i, name in enumerate(ACTION_FIELDS)}
    action_values["delay_queue"] = rows(62).reshape(n,2,31)
    action_values["delay_steps"] = ids % 2
    m.action_calls = m.target_calls = 0
    def action_capture(selected):
        m.action_calls += 1
        return {name: value[selected].clone() for name, value in action_values.items()}
    target_values = {f"field_{i}": rows(3) + i for i in range(target_fields)}
    target_values["history"] = rows(12).reshape(n,2,2,3)
    target_values["mask"] = ids % 3 == 0
    target_values["sentinel"] = torch.full((n,), float("nan"), device=device)
    def target_capture(selected):
        m.target_calls += 1
        return {name: value[selected].clone() for name, value in target_values.items()}
    action = SimpleNamespace(action_dim=31, _qdes_delay_max_steps=1,
        capture_markov_replay_state=action_capture, restore_markov_replay_state=lambda *_: None)
    manager = SimpleNamespace(active_terms=["joint_pos"], action=rows(31)+5, prev_action=rows(31)+6,
                              get_term=lambda _: action)
    m._env = SimpleNamespace(action_manager=manager,
                            scene=SimpleNamespace(env_origins=rows(3)/100))
    m._v17_target_term = lambda: SimpleNamespace(capture_markov_replay_state=target_capture)
    m.post_swing_capture_enabled = lambda: m._post_swing_ability_unlocked
    m._replay_severity = lambda selected, _: ((selected // 2) % 3, selected % 11 != 0)
    for name, function in production_methods(source).items():
        setattr(m, name, MethodType(function, m))
    m._update_v17_replay_metrics = lambda: None
    m._ensure_v17_replay_storage()
    return m, action_values, target_values


def assert_provenance(m, action, target):
    for bucket in range(24):
        for slot in range(m._v17_replay_bucket_capacity):
            env = int(m._v17_replay_source_env[bucket, slot])
            if env < 0:
                continue
            assert bucket // 12 == int(m.clip_id[env])
            for name, values in action.items():
                torch.testing.assert_close(getattr(m, "_v17_replay_action_"+name)[bucket,slot], values[env])
            for name, values in target.items():
                torch.testing.assert_close(m._v17_replay_target_state[name][bucket,slot], values[env], equal_nan=True)
            assert int(m._v17_replay_rally_wrap_step[bucket,slot]) == int(m.rally_wrap_step[env])


@pytest.mark.parametrize("device", ["cpu", pytest.param("cuda:1", marks=pytest.mark.skipif(
    not torch.cuda.is_available() or torch.cuda.device_count()<2, reason="second GPU unavailable"))])
def test_cross_bucket_snapshot_preserves_every_field_and_overwrite_pointer(device):
    m, action, target = fixture(device=device)
    for selected in [torch.arange(36, device=device), torch.arange(7,25, device=device),
                     torch.arange(0,36,2, device=device)]:
        m._capture_post_swing_states(selected, phase_bin=1)
        assert_provenance(m, action, target)
        # Every advertised same-plant slot must still be owned by that plant.
        bucket, env = torch.where(m._v17_replay_local_latest_slot >= 0)
        slots = m._v17_replay_local_latest_slot[bucket,env]
        assert torch.equal(m._v17_replay_source_env[bucket,slots], env)
        assert m._v17_replay_bucket_count_host == m._v17_replay_bucket_count.tolist()
    assert m.action_calls == m.target_calls == 3  # one full snapshot per call, not per bucket


def test_over_capacity_slice_matches_original_admission_and_does_not_consume_rng():
    m, _, _ = fixture(n=120, capacity=3)
    ids = torch.arange(120)
    state = torch.random.get_rng_state()
    m._capture_post_swing_states(ids, phase_bin=0)
    assert torch.equal(state, torch.random.get_rng_state())
    for bucket in [0,1,2,12,13,14]:
        legal = ids[(ids % 11 != 0) & ((ids % 2)*12 + (ids//2)%3 == bucket)]
        expected = legal[torch.floor(torch.arange(3).float()*len(legal)/3).long()]
        assert torch.equal(m._v17_replay_source_env[bucket], expected)


def test_wrap_clock_is_clamped_but_illegal_hot_clock_is_rejected():
    m, _, _ = fixture()
    m.time_steps[:] = m.clip_id * 100 + 100
    with pytest.raises(RuntimeError, match="clock"):
        m._capture_post_swing_states(torch.arange(36), phase_bin=0)
    m._capture_post_swing_states(torch.arange(36), phase_bin=3)
    valid = m._v17_replay_source_env >= 0
    ids = m._v17_replay_source_env[valid]
    assert torch.equal(m._v17_replay_motion_time_steps[valid], m.rally_wrap_step[ids]-1)


def test_legacy_metadata_repair_and_gate_clear():
    m, _, _ = fixture()
    m._capture_post_swing_states(torch.arange(36), phase_bin=0)
    m._v17_replay_bucket_count_host = None
    m._v17_replay_ready_latched = False
    assert m._v17_replay_host_counts() == m._v17_replay_bucket_count.tolist()
    assert m._v17_replay_ready()
    m._v17_replay_last_sample_bucket = torch.zeros(36, dtype=torch.long)
    m._v17_replay_last_sample_slot = torch.zeros(36, dtype=torch.long)
    m._v17_replay_pending_target_mask = torch.ones(36, dtype=torch.bool)
    m._v17_replay_failure_score = torch.ones(24)
    m._v17_replay_pending_failure_count = torch.ones(24)
    m._update_v17_failure_sampling = lambda: None
    m._clear_post_swing_replay_bank()
    assert not m._v17_replay_ready()
    assert m._v17_replay_host_counts() == [0]*24


def test_first_ready_capture_starts_ability_ramp_on_the_same_control_tick():
    m, _, _ = fixture()
    m._post_swing_ability_gate_enabled = True
    m._post_swing_replay_ready_step = -1
    m._post_swing_replay_ramp_probabilities = [0.1, 0.35]
    m.cfg.post_swing_replay_ramp_interval_steps = 100
    m._env.common_step_counter = 123
    m._capture_post_swing_states(torch.arange(36), phase_bin=0)
    assert m._post_swing_replay_ready_step == 123
    m._env.common_step_counter = 223
    assert m._ability_gated_post_probability() == (0.35, 1)
