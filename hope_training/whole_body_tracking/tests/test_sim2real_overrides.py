"""The public sim-to-real YAML changes real event and actor config fields."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "sim2real_overrides", ROOT / "source/whole_body_tracking/whole_body_tracking/utils/task_sim2real_overrides.py"
)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def env_config():
    names = ("base_ang_vel", "joint_pos", "joint_vel", "projected_gravity", "base_forward_xy",
             "base_target_delta_xy", "racket_target_rel_base")
    policy = NS(enable_corruption=False, **{name: NS(noise=NS(n_min=-1, n_max=1)) for name in names})
    return NS(
        observations=NS(policy=policy, critic=NS(enable_corruption=False, joint_vel=NS(noise=None))),
        events=NS(base_com=NS(params={}), physics_material=NS(params={}),
                  sim2real_push=NS(params={}, interval_range_s=(1, 3))),
    )


def test_published_recipe_sets_actor_noise_and_plant_parameters():
    task = yaml.safe_load((ROOT / "cfg/task/HOPEPingPong.yaml").read_text())
    env = env_config()
    mod.apply_sim2real_overrides(env, task, [])
    assert env.observations.policy.enable_corruption
    assert env.observations.policy.joint_vel.noise.n_max == 0.5
    assert env.observations.policy.base_target_delta_xy.noise.n_max == 0
    assert not env.observations.critic.enable_corruption
    assert env.observations.critic.joint_vel.noise is None
    assert env.events.sim2real_push.params["cohort_fraction"] == 0.5
    assert env.events.sim2real_push.interval_range_s == (5, 8)
    assert env.events.base_com.params["com_range"]["y"] == (-0.05, 0.05)
    assert env.events.physics_material.params["restitution_range"] == (0, 0.1)
    assert env.events.physics_material.params["make_consistent"] is True


@pytest.mark.parametrize("task", [
    {"observation_noise": {"joint_poss": [0, 1]}},
    {"observation_noise": {"enabled": "false"}},
    {"observation_noise": {"joint_pos": [0, float("nan")]}},
    {"domain_rand": {"push": {"cohort_fraction": 1.5}}},
    {"domain_rand": {"push": {"interval_s": [0, 1]}}},
    {"domain_rand": {"push": {"ramp_steps": 0.5}}},
])
def test_bad_knobs_fail_before_simulation(task):
    with pytest.raises(ValueError):
        mod.apply_sim2real_overrides(env_config(), task, [])


def test_no_new_block_leaves_existing_configuration_intact():
    env = env_config()
    mod.apply_sim2real_overrides(env, {}, [])
    assert env.events.sim2real_push.params == {}
    assert env.observations.policy.joint_vel.noise.n_max == 1
