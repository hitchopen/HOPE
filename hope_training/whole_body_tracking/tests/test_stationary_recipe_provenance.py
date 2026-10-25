"""Preserve the published training strategy while changing its physics/bank binding.

Fingerprints were captured independently from both the fully resolved source
nohope/main_improve_2@9c4d6366ab7bd58b73baf98c06159e1ac51688d2 and PR 87's
standalone recipe at 3a19636a0dadc79cd985a45d0890007c8d0d2222. They agreed:
full task 3dcea64afd8a292e141c1d6dedd7b2d6ecdca237347dd45e9045d488aba652de;
algorithm 1fa13da751dfafa2d00be10776d0e0733a1e254db8778a48a6f4a6b16e15d9a7.

The strategy fingerprint below removes only the upstream ball_physics block and
five bank-identity fields, then hashes canonical resolved JSON. Its reward,
pose, curriculum, timing, action and station settings remain part of the hash.
The separate PPQ004 default and its full-model recipe-22 identity are covered by
test_ppq004_training_default.py; this test does not restore recipe-21 labels.
"""

import copy
import hashlib
import json
from pathlib import Path

from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf


ROOT = Path(__file__).resolve().parents[1]
BANK_IDENTITY_KEYS = (
    "venue_tuple_bank_path",
    "venue_tuple_bank_receipt_path",
    "venue_tuple_bank_sha256",
    "venue_tuple_bank_receipt_sha256",
    "venue_tuple_bank_recipe_revision",
)
SOURCE_STRATEGY_SHA256 = "d38c18391cc859e1a41203f8a6f6f85b30b92b804393956792e8221db5d0e50b"
SOURCE_ALGORITHM_SHA256 = "1fa13da751dfafa2d00be10776d0e0733a1e254db8778a48a6f4a6b16e15d9a7"


def resolved_recipe():
    with initialize_config_dir(config_dir=str(ROOT / "cfg"), version_base=None):
        cfg = compose(config_name="train", overrides=[
            "task=HOPEPingPongStationaryRally", "algo=ppo_balanced_hold",
        ])
        return OmegaConf.to_container(cfg, resolve=True)


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def test_inherited_stationary_strategy_matches_pr87_and_published_source():
    task = copy.deepcopy(resolved_recipe()["task"])
    task.pop("ball_physics", None)
    for key in BANK_IDENTITY_KEYS:
        del task["racket"][key]
    assert canonical_sha256(task) == SOURCE_STRATEGY_SHA256


def test_balanced_ppo_matches_pr87_and_published_source():
    assert canonical_sha256(resolved_recipe()["algo"]) == SOURCE_ALGORITHM_SHA256
