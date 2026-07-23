"""Resolved-configuration tests for the domain-randomization CLI overrides.

Finding: the trainer's override code targeted ``events.randomize_link_mass`` /
``events.randomize_pd_gains``, but the environment configuration defines the terms as
``events.link_mass`` and ``events.pd_gains`` — every requested range was silently
ignored. These tests apply ``_apply_domain_rand`` from ``scripts/train.py`` to a
stand-in events config using the REAL field names and cover: enabled override,
disabled (null) range, absent key (keep default), and an env cfg without the term.

``train.py`` imports hydra/omegaconf at module scope; light stubs are injected so
the function under test loads without those packages (they are not needed by it).

Run:  python tests/test_domain_rand_overrides.py   (or pytest)
"""

from __future__ import annotations

import importlib.util
import os
import sys
import tempfile
import types
from pathlib import Path
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
    """Stand-in for HOPEPingPongEnvCfg.events with the REAL term names."""
    return SimpleNamespace(
        link_mass=SimpleNamespace(
            params={"mass_distribution_params": (0.9, 1.1), "operation": "scale"}
        ),
        pd_gains=SimpleNamespace(
            params={
                "stiffness_distribution_params": (0.9, 1.1),
                "damping_distribution_params": (0.9, 1.1),
            }
        ),
    )


def test_enabled_override_applies_to_real_field_names():
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(
        env_cfg, {"link_mass_range": [0.8, 1.2], "pd_gain_range": [0.7, 1.3]}, applied
    )
    assert env_cfg.events.link_mass.params["mass_distribution_params"] == (0.8, 1.2)
    assert env_cfg.events.pd_gains.params["stiffness_distribution_params"] == (0.7, 1.3)
    assert env_cfg.events.pd_gains.params["damping_distribution_params"] == (0.7, 1.3)
    assert len(applied) == 2, f"both overrides must be reported as applied: {applied}"


def test_null_range_disables_the_event():
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(env_cfg, {"link_mass_range": None, "pd_gain_range": None}, applied)
    assert env_cfg.events.link_mass is None, "null link_mass_range must disable the event"
    assert env_cfg.events.pd_gains is None, "null pd_gain_range must disable the event"
    assert len(applied) == 2


def test_absent_key_keeps_the_default():
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(env_cfg, {"pd_gain_range": [0.85, 1.15]}, applied)
    # link_mass untouched at its default; pd_gains overridden.
    assert env_cfg.events.link_mass.params["mass_distribution_params"] == (0.9, 1.1)
    assert env_cfg.events.pd_gains.params["stiffness_distribution_params"] == (0.85, 1.15)
    assert applied == ["events.pd_gains = (0.85, 1.15)"]


def test_range_on_already_disabled_event_warns_not_crashes():
    env_cfg = SimpleNamespace(events=SimpleNamespace(link_mass=None, pd_gains=None))
    applied: list = []
    train._apply_domain_rand(
        env_cfg, {"link_mass_range": [0.8, 1.2], "pd_gain_range": [0.8, 1.2]}, applied
    )
    assert env_cfg.events.link_mass is None and env_cfg.events.pd_gains is None
    assert applied == []


def test_none_domain_rand_is_a_noop():
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(env_cfg, None, applied)
    assert env_cfg.events.link_mass.params["mass_distribution_params"] == (0.9, 1.1)
    assert applied == []


def test_default_task_yaml_knobs_resolve_against_real_event_names():
    """The shipped randomization_base defaults must hit real fields (no silent no-op)."""
    import yaml

    with open(os.path.join(_ROOT, "cfg", "base", "randomization_base.yaml")) as f:
        base = yaml.safe_load(f)
    dr = base["domain_rand"]
    env_cfg = SimpleNamespace(events=_events_cfg())
    applied: list = []
    train._apply_domain_rand(env_cfg, dr, applied)
    # Old HOPE defaults apply: link_mass +/-15%, PD gains +/-20%.
    assert env_cfg.events.link_mass.params["mass_distribution_params"] == (0.85, 1.15)
    assert env_cfg.events.pd_gains.params["stiffness_distribution_params"] == (0.8, 1.2)
    assert env_cfg.events.pd_gains.params["damping_distribution_params"] == (0.8, 1.2)
    assert len(applied) == 2


def test_motion_manifest_resolves_many_clips_and_metadata():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        clips = []
        for idx in range(3):
            clip = root / f"clip_{idx}.npz"
            clip.write_bytes(b"placeholder")
            clips.append(clip)
        manifest = root / "manifest.tsv"
        manifest.write_text(
            "output\tframes\tstrike_frame\tstrike_phase\tswing_side\t"
            "racket_pos_x_lo\tracket_pos_x_hi\tracket_pos_y_lo\tracket_pos_y_hi\t"
            "racket_pos_z_lo\tracket_pos_z_hi\n"
            f"{clips[0]}\t40\t10\t0.256410256\t1\t0.40\t0.50\t-0.30\t-0.20\t1.10\t1.20\n"
            f"{clips[1]}\t50\t20\t0.408163265\t1\t0.45\t0.55\t-0.25\t-0.15\t1.15\t1.25\n"
            f"{clips[2]}\t60\t30\t0.508474576\t-1\t0.50\t0.60\t0.15\t0.25\t1.20\t1.30\n",
            encoding="utf-8",
        )
        cfg = SimpleNamespace(motion_manifest=str(manifest))

        files, metadata = train._resolve_motion_plan(cfg)

        assert files == [str(path.resolve()) for path in clips]
        assert [m["swing_side"] for m in metadata] == [1.0, 1.0, -1.0]
        assert [round(m["strike_phase"], 6) for m in metadata] == [0.25641, 0.408163, 0.508475]
        assert metadata[0]["racket_pos_range"] == ((0.40, 0.50), (-0.30, -0.20), (1.10, 1.20))


def test_motion_metadata_applies_per_clip_timing_and_side_boxes():
    forehand_box = ((0.45, 0.55), (-0.55, -0.15), (0.70, 1.00))
    backhand_box = ((0.45, 0.55), (0.15, 0.55), (0.85, 1.15))
    forehand_vel = ((1.0, 2.0), (0.5, 1.5), (0.2, 1.0))
    backhand_vel = ((1.5, 2.5), (-1.5, -0.5), (0.0, 0.7))
    racket = SimpleNamespace(
        strike_phase_per_clip=(0.5, 0.5),
        swing_side_per_clip=(),
        mount_normal_sign_per_clip=(1.0, -1.0),
        racket_pos_range_per_clip=(forehand_box, backhand_box),
        racket_vel_range_per_clip=(forehand_vel, backhand_vel),
    )
    env_cfg = SimpleNamespace(commands=SimpleNamespace(racket_target=racket))
    metadata = [
        {"strike_phase": 0.25, "swing_side": 1.0},
        {"strike_phase": 0.50, "swing_side": 1.0},
        {"strike_phase": 0.75, "swing_side": -1.0},
    ]
    applied: list = []

    train._apply_motion_metadata(env_cfg, ["a.npz", "b.npz", "c.npz"], metadata, applied)

    assert racket.strike_phase_per_clip == (0.25, 0.50, 0.75)
    assert racket.swing_side_per_clip == (1.0, 1.0, -1.0)
    assert racket.mount_normal_sign_per_clip == (1.0, 1.0, -1.0)
    assert racket.racket_pos_range_per_clip == (forehand_box, forehand_box, backhand_box)
    assert racket.racket_vel_range_per_clip == (forehand_vel, forehand_vel, backhand_vel)
    assert any("strike_phase_per_clip" in line for line in applied)


def test_motion_metadata_explicit_target_boxes_override_side_defaults():
    default_forehand_box = ((0.45, 0.55), (-0.55, -0.15), (0.70, 1.00))
    default_backhand_box = ((0.45, 0.55), (0.15, 0.55), (0.85, 1.15))
    explicit_a = ((0.50, 0.58), (-0.22, -0.14), (1.25, 1.33))
    explicit_b = ((0.62, 0.70), (-0.10, -0.02), (1.34, 1.42))
    explicit_vel_a = ((0.8, 1.1), (2.0, 2.4), (1.4, 1.8))
    explicit_vel_b = ((0.9, 1.2), (2.1, 2.5), (1.5, 1.9))
    racket = SimpleNamespace(
        strike_phase_per_clip=(),
        swing_side_per_clip=(),
        mount_normal_sign_per_clip=(),
        racket_pos_range_per_clip=(default_forehand_box, default_backhand_box),
        racket_vel_range_per_clip=None,
    )
    env_cfg = SimpleNamespace(commands=SimpleNamespace(racket_target=racket))
    metadata = [
        {"strike_phase": 0.25, "swing_side": 1.0, "racket_pos_range": explicit_a, "racket_vel_range": explicit_vel_a},
        {"strike_phase": 0.50, "swing_side": 1.0, "racket_pos_range": explicit_b, "racket_vel_range": explicit_vel_b},
    ]
    applied: list = []

    train._apply_motion_metadata(env_cfg, ["a.npz", "b.npz"], metadata, applied)

    assert racket.racket_pos_range_per_clip == (explicit_a, explicit_b)
    assert racket.racket_vel_range_per_clip == (explicit_vel_a, explicit_vel_b)
    assert any("racket_pos_range_per_clip = from motion metadata" in line for line in applied)


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
