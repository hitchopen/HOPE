"""Canonical table-region identity and frame conversion require no simulator."""
from copy import deepcopy
import hashlib
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from ball_physics_profile import load_profile
from sweet_spot_profile import load_objective, objective_sha256, objective_in_training_frame

OBJECTIVE = REPO / 'configs/return_placement_sweet_spot.yaml'
PHYSICS = REPO / 'configs/ball_physics_ppq004_training.yaml'


def test_page_two_opponent_rectangle_and_training_frame_are_distinct_from_bounce_height():
    descriptor = load_objective(OBJECTIVE)
    assert descriptor == dict(schema_version=1, id='table_return_sweet_spot_v1',
        frame='table_near_left', units='m', x_range=[2.24,2.44], y_range=[-.575,-.425],
        surface_z=0., target_policy='fixed_center', boundary='inclusive', target_xy=[2.34,-.5])
    assert objective_sha256(OBJECTIVE) == hashlib.sha256(OBJECTIVE.read_bytes()).hexdigest()
    profile = load_profile(PHYSICS)
    transformed = objective_in_training_frame(descriptor, profile)
    assert transformed['x_range'] == pytest.approx([2.74,2.94])
    assert transformed['y_range'] == pytest.approx([.1875,.3375])
    assert transformed['target_xy'] == pytest.approx([2.84,.2625])
    assert transformed['surface_z'] == .76  # tabletop; full model adds radius independently
    # Translation comes from selected geometry, never the familiar default constants.
    moved = deepcopy(profile)
    moved['training_frame'].update(near_x=.7, y_center=-.1, surface_z=.9)
    moved['table']['width'] = 1.6
    shifted = objective_in_training_frame(descriptor, moved)
    assert shifted['target_xy'] == pytest.approx([3.04,.2])
    assert shifted['surface_z'] == .9


@pytest.mark.parametrize('key,value', [
    ('schema_version', True), ('schema_version', 2), ('frame', 'training_env_local'),
    ('units', 'cm'), ('x_range', [2.44,2.24]), ('y_range', [float('nan'),0]),
    ('x_range', [True,3]), ('surface_z', .02), ('surface_z', False),
    ('target_policy', 'uniform'), ('boundary', 'exclusive'),
    ('target_xy', [2.34,-.5]), ('reward_weight', 10),
])
def test_profile_rejects_ambiguous_or_unversioned_objective_fields(tmp_path, key, value):
    raw = yaml.safe_load(OBJECTIVE.read_text())
    raw[key] = value
    path = tmp_path/'objective.yaml'
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError):
        load_objective(path)


@pytest.mark.parametrize('key,bounds', [('x_range',[.3,.5]), ('x_range',[2.7,2.8]),
                                      ('y_range',[-1.6,-1.4]), ('y_range',[-.1,.1])])
def test_objective_must_remain_on_opponent_table(tmp_path, key, bounds):
    raw = yaml.safe_load(OBJECTIVE.read_text())
    raw[key] = bounds
    path = tmp_path/'objective.yaml'
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='opponent tabletop'):
        objective_in_training_frame(load_objective(path), load_profile(PHYSICS))


def test_canonical_target_cannot_disagree_with_rectangle_and_hash_detects_geometry_change(tmp_path):
    descriptor = load_objective(OBJECTIVE)
    descriptor['target_xy'][0] += .01
    with pytest.raises(ValueError, match='rectangle centre'):
        objective_in_training_frame(descriptor, load_profile(PHYSICS))
    copied = tmp_path/'objective.yaml'
    copied.write_bytes(OBJECTIVE.read_bytes())
    assert load_objective(copied) == load_objective(OBJECTIVE)
    assert objective_sha256(copied) == objective_sha256(OBJECTIVE)
    raw = yaml.safe_load(copied.read_text())
    raw['x_range'][0] += .01
    copied.write_text(yaml.safe_dump(raw))
    assert objective_sha256(copied) != objective_sha256(OBJECTIVE)
