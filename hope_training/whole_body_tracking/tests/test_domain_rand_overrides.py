"""Resolved-configuration tests for the domain-randomization CLI overrides.

The env configuration (``HOPEEventCfg`` in
``tasks/tracking/config/agibot_a3/hope_env_cfg.py``) names the terms
``events.randomize_link_mass`` and ``events.randomize_pd_gains`` — the trainer's
``_apply_domain_rand`` (``scripts/train.py``) must target those exact fields. These
tests apply the function to a stand-in events config using the REAL field names and
cover: enabled override, disabled (null) range, absent key (keep default), an env
cfg without the term, and the shipped a3-message PD mode
(``pd_mode: a3_message_passive_nominal_cohort_v1``): under it ``pd_gain_range: null``
must NOT disable the installed ``randomize_a3_message_pd_gains`` term — the
``pd_alpha_range`` / ``pd_beta_range`` / ``pd_nominal_fraction`` knobs refine it.

``train.py`` imports hydra/omegaconf at module scope; light stubs are injected so
the function under test loads without those packages (they are not needed by it).

Run:  python tests/test_domain_rand_overrides.py   (or pytest)
"""

from __future__ import annotations

import importlib.util
import os
import sys
import types
from types import SimpleNamespace

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TRAIN_PY = os.path.join(_ROOT, "scripts", "train.py")


def _install_stubs() -> None:
    if "hydra" not in sys.modules:
        hydra = types.ModuleType("hydra")
        hydra.main = lambda **_kw: (lambda fn: fn)
        sys.modules["hydra"] = hydra
    if "omegaconf" not in sys.modules:
        omegaconf = types.ModuleType("omegaconf")

        class _OmegaConf:  # only the attributes train.py touches at call time
            @staticmethod
            def to_container(cfg, resolve=False):
                return dict(cfg)

            @staticmethod
            def resolve(cfg):
                return cfg

            @staticmethod
            def set_struct(cfg, flag):
                return cfg

        omegaconf.OmegaConf = _OmegaConf
        sys.modules["omegaconf"] = omegaconf


def _load_train():
    _install_stubs()
    spec = importlib.util.spec_from_file_location("hope_train_script", _TRAIN_PY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


train = _load_train()


def _events_cfg():
    """Stand-in for HOPEEventCfg with the REAL term names (generic scale PD term)."""
    return SimpleNamespace(
        randomize_link_mass=SimpleNamespace(
            params={"mass_distribution_params": (0.85, 1.15), "operation": "scale"}
        ),
        randomize_pd_gains=SimpleNamespace(
            params={
                "stiffness_distribution_params": (0.8, 1.2),
                "damping_distribution_params": (0.8, 1.2),
            }
        ),
    )


def _events_cfg_a3_message():
    """Stand-in for the HitterPingPong recipe: randomize_a3_message_pd_gains installed."""
    return SimpleNamespace(
        randomize_link_mass=SimpleNamespace(
            params={"mass_distribution_params": (0.85, 1.15), "operation": "scale"}
        ),
        randomize_pd_gains=SimpleNamespace(
            params={
                "alpha_range": (0.85, 1.15),
                "beta_range": (0.85, 1.15),
                "nominal_fraction": 0.25,
            }
        ),
    )


def test_enabled_override_applies_to_real_field_names():
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(
        env_cfg, {"link_mass_range": [0.8, 1.2], "pd_gain_range": [0.7, 1.3]}, applied
    )
    assert env_cfg.events.randomize_link_mass.params["mass_distribution_params"] == (0.8, 1.2)
    assert env_cfg.events.randomize_pd_gains.params["stiffness_distribution_params"] == (0.7, 1.3)
    assert env_cfg.events.randomize_pd_gains.params["damping_distribution_params"] == (0.7, 1.3)
    assert len(applied) == 2, f"both overrides must be reported as applied: {applied}"


def test_null_range_disables_the_event():
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(env_cfg, {"link_mass_range": None, "pd_gain_range": None}, applied)
    assert env_cfg.events.randomize_link_mass is None, "null link_mass_range must disable the event"
    assert env_cfg.events.randomize_pd_gains is None, "null pd_gain_range must disable the event"
    assert len(applied) == 2


def test_absent_key_keeps_the_default():
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(env_cfg, {"pd_gain_range": [0.85, 1.15]}, applied)
    # link mass untouched at its default; pd gains overridden.
    assert env_cfg.events.randomize_link_mass.params["mass_distribution_params"] == (0.85, 1.15)
    assert env_cfg.events.randomize_pd_gains.params["stiffness_distribution_params"] == (0.85, 1.15)
    assert applied == ["events.randomize_pd_gains = (0.85, 1.15)"]


def test_range_on_already_disabled_event_warns_not_crashes():
    env_cfg = SimpleNamespace(events=SimpleNamespace(randomize_link_mass=None, randomize_pd_gains=None))
    applied: list = []
    train._apply_domain_rand(
        env_cfg, {"link_mass_range": [0.8, 1.2], "pd_gain_range": [0.8, 1.2]}, applied
    )
    assert env_cfg.events.randomize_link_mass is None and env_cfg.events.randomize_pd_gains is None
    assert applied == []


def test_none_domain_rand_is_a_noop():
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(env_cfg, None, applied)
    assert env_cfg.events.randomize_link_mass.params["mass_distribution_params"] == (0.85, 1.15)
    assert applied == []


def test_a3_message_pd_mode_null_range_keeps_the_term():
    """pd_gain_range: null under the a3-message mode retires only the generic scale DR."""
    env_cfg = SimpleNamespace(events=_events_cfg_a3_message())
    applied: list = []
    train._apply_domain_rand(
        env_cfg,
        {
            "link_mass_range": [0.85, 1.15],
            "pd_gain_range": None,
            "pd_mode": "a3_message_passive_nominal_cohort_v1",
            "pd_alpha_range": [0.9, 1.1],
            "pd_beta_range": [0.8, 1.2],
            "pd_nominal_fraction": 0.5,
        },
        applied,
    )
    term = env_cfg.events.randomize_pd_gains
    assert term is not None, "a3-message PD term must NOT be disabled by pd_gain_range: null"
    assert term.params["alpha_range"] == (0.9, 1.1)
    assert term.params["beta_range"] == (0.8, 1.2)
    assert term.params["nominal_fraction"] == 0.5
    assert len(applied) == 4  # link mass + alpha + beta + nominal


def test_default_base_yaml_knobs_resolve_against_real_event_names():
    """The shipped randomization_base defaults must hit real fields (no silent no-op)."""
    import yaml

    with open(os.path.join(_ROOT, "cfg", "base", "randomization_base.yaml")) as f:
        base = yaml.safe_load(f)
    dr = base["domain_rand"]
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(env_cfg, dr, applied)
    # link_mass_range [0.85, 1.15] and pd_gain_range [0.8, 1.2] both apply (no pd_mode in base).
    assert env_cfg.events.randomize_link_mass.params["mass_distribution_params"] == (0.85, 1.15)
    assert env_cfg.events.randomize_pd_gains.params["stiffness_distribution_params"] == (0.8, 1.2)
    assert len(applied) == 2


def test_hitter_pingpong_task_yaml_resolves_the_a3_message_recipe():
    """The shipped HitterPingPong domain_rand (merged over the base) refines, never disables."""
    import yaml

    with open(os.path.join(_ROOT, "cfg", "base", "randomization_base.yaml")) as f:
        dr = yaml.safe_load(f)["domain_rand"]
    with open(os.path.join(_ROOT, "cfg", "task", "HOPEPingPong.yaml")) as f:
        task_dr = yaml.safe_load(f)["domain_rand"]
    dr.update(task_dr)  # Hydra-style leaf merge of the task over the base
    assert dr["pd_mode"] == "a3_message_passive_nominal_cohort_v1"
    assert dr["pd_gain_range"] is None

    env_cfg = SimpleNamespace(events=_events_cfg_a3_message())
    applied: list = []
    train._apply_domain_rand(env_cfg, dr, applied)
    assert env_cfg.events.randomize_link_mass.params["mass_distribution_params"] == (0.85, 1.15)
    term = env_cfg.events.randomize_pd_gains
    assert term is not None
    assert term.params["alpha_range"] == (0.85, 1.15)
    assert term.params["beta_range"] == (0.85, 1.15)
    assert term.params["nominal_fraction"] == 0.25


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"[ok] {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"[FAIL] {fn.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} domain-rand override tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
