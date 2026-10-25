"""Recipe-23 identity, default enablement and same-bank reward ablations."""
import ast
from copy import deepcopy
import json
from types import SimpleNamespace

import pytest
import numpy as np
import yaml
from omegaconf import OmegaConf

from test_ppq004_training_default import config
from test_refitted_stationary_launcher import ROOT, bundle, digest, launcher, compose_launch
from test_sim2real_export import module as exporter, fixture as export_fixture
from test_command_overrides import module as commands
from test_complete_hold_robustness import robust
import ppq004_training as prelaunch
from sweet_spot_profile import load_objective, objective_sha256


@pytest.fixture
def sweet_bundle(bundle):
    manifest, bank, physics, receipt = bundle
    objective = manifest.parent / 'sweet_spot.yaml'
    objective.write_bytes((ROOT.parents[1] / 'configs/return_placement_sweet_spot.yaml').read_bytes())
    identity = dict(path=str(objective), sha256=objective_sha256(objective), descriptor=load_objective(objective))
    data = json.loads(receipt.read_text())
    data['contract'].update(recipe_revision=23, landing_objective=identity['descriptor'],
                            landing_objective_sha256=identity['sha256'], landing_x=[2.2,3.1], landing_y=[-.65,.65],
                            training_distribution_revision=prelaunch.SWEET_SPOT_DISTRIBUTION)
    data['source'].update(landing_objective_yaml=str(objective), landing_objective_yaml_sha256=identity['sha256'])
    receipt.write_text(json.dumps(data))
    data = yaml.safe_load(manifest.read_text())
    data['task']['landing_objective'] = identity
    data['task']['racket'].update(venue_tuple_bank_recipe_revision=23, venue_tuple_bank_receipt_sha256=digest(receipt))
    manifest.write_text(yaml.safe_dump(data))
    return manifest, bank, physics, receipt


def resolved(bundle, *overrides):
    cfg = config([f'task.ppq004_refit_manifest={json.dumps(str(bundle[0]))}', *overrides])
    prelaunch.configure_refitted_task(cfg, environ={})
    return OmegaConf.to_container(cfg, resolve=True)


def test_default_on_and_off_use_identical_questions_and_other_rewards(sweet_bundle):
    enabled = resolved(sweet_bundle)
    disabled = resolved(sweet_bundle, 'task.sweet_spot.reward_enabled=false')
    for cfg in (enabled, disabled):
        robust.validate_training_contract(cfg['task'], cfg['algo'])
        task = cfg['task']
        assert task['build_contract']['training_distribution_revision'] == prelaunch.SWEET_SPOT_DISTRIBUTION
        assert task['build_contract']['reward_revision'] == prelaunch.SWEET_SPOT_REWARD
        assert task['racket']['sweet_spot_x_range'] == pytest.approx([2.74, 2.94])
        assert task['racket']['sweet_spot_y_range'] == pytest.approx([.1875, .3375])
    on, off = enabled['task'], disabled['task']
    assert on['landing_objective'] == off['landing_objective']
    assert all(on['racket'][key] == off['racket'][key] for key in prelaunch.BANK_KEYS)
    assert on['rewards']['sweet_spot_landing_weight'] == 30.0
    assert off['rewards']['sweet_spot_landing_weight'] == 0.0
    assert {key:value for key,value in on['rewards'].items() if key != 'sweet_spot_landing_weight'} == {
        key:value for key,value in off['rewards'].items() if key != 'sweet_spot_landing_weight'}
    assert on['motion'] == off['motion'] and on['action'] == off['action']


def test_launcher_only_accepts_a_plain_boolean_for_same_bank_ablation(sweet_bundle, bundle):
    command, _ = launcher.prepare_launch(sweet_bundle[0], ['task.sweet_spot.reward_enabled=false'])
    cfg = compose_launch(command)
    assert not cfg['task']['sweet_spot']['reward_enabled']
    assert cfg['task']['racket']['venue_tuple_bank_recipe_revision'] == 23
    for argument in ('task.sweet_spot.reward_enabled=0', 'task.sweet_spot.reward_enabled="false"',
                     'task.sweet_spot.reward_enabled=true,false', '++task.sweet_spot.reward_enabled=false',
                     'task.sweet_spot.reward_enabled=${oc.env:FLAG}', 'task.sweet_spot.reward_weight=10'):
        with pytest.raises(ValueError, match='ordinary|boolean'):
            launcher.prepare_launch(sweet_bundle[0], [argument])


@pytest.mark.parametrize('override', [
    'task.sweet_spot.reward_enabled=1', 'task.sweet_spot.distance_scale_m=0',
    'task.sweet_spot.inside_bonus=-1', 'task.sweet_spot.reward_weight=0',
    'task.racket.sweet_spot_x_range=[0,1]', 'task.racket.sweet_spot_reward_enabled=false',
    'task.rewards.sweet_spot_landing_weight=7',
])
def test_invalid_or_conflicting_reward_controls_fail_before_isaac(sweet_bundle, override):
    with pytest.raises(ValueError, match='boolean|range|weight|conflicts'):
        resolved(sweet_bundle, override)


def test_general_recipe22_is_explicit_and_cannot_be_substituted_for23(bundle, sweet_bundle):
    # sweet_bundle shares the fixture's files, so restore a separate legacy shape here.
    manifest, _, _, receipt = sweet_bundle
    raw = json.loads(receipt.read_text())
    raw['contract']['recipe_revision'] = 22
    raw['contract'].pop('landing_objective')
    raw['contract'].pop('landing_objective_sha256')
    receipt.write_text(json.dumps(raw))
    raw = yaml.safe_load(manifest.read_text())
    raw['task'].pop('landing_objective')
    raw['task']['racket'].update(venue_tuple_bank_recipe_revision=22, venue_tuple_bank_receipt_sha256=digest(receipt))
    manifest.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='matching'):
        resolved(bundle)
    command, _ = launcher.prepare_launch(manifest)
    assert 'task=HOPEPingPongStationaryPPQ004General' in command
    cfg = compose_launch(command)
    assert 'sweet_spot' not in cfg['task']
    assert cfg['task']['build_contract']['training_distribution_revision'] == prelaunch.DISTRIBUTION
    with pytest.raises(ValueError, match='ordinary run options'):
        launcher.prepare_launch(manifest, ['task.sweet_spot.reward_enabled=false'])


@pytest.mark.parametrize('field', ['objective_hash', 'objective_descriptor', 'source_hash', 'landing_bounds'])
def test_objective_receipt_mismatch_is_rejected_even_with_updated_receipt_hash(sweet_bundle, field):
    manifest, _, _, receipt = sweet_bundle
    raw = json.loads(receipt.read_text())
    if field == 'objective_hash':
        raw['contract']['landing_objective_sha256'] = '0'*64
    elif field == 'objective_descriptor':
        raw['contract']['landing_objective']['target_xy'][0] += .1
    elif field == 'source_hash':
        raw['source']['landing_objective_yaml_sha256'] = '0'*64
    else:
        raw['contract']['landing_x'] = [2.0, 2.5]
    receipt.write_text(json.dumps(raw))
    raw = yaml.safe_load(manifest.read_text())
    raw['task']['racket']['venue_tuple_bank_receipt_sha256'] = digest(receipt)
    manifest.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='objective|landing bounds'):
        launcher.prepare_launch(manifest)


def test_resume_rejects_changed_switch_region_scale_bonus_weight_or_bank(sweet_bundle):
    task = resolved(sweet_bundle)['task']
    identity = exporter.placement_reward_identity(task)
    tree = ast.parse((ROOT/'scripts/train.py').read_text())
    node = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name == '_validate_training_checkpoint')
    scope = {}
    exec(compile(ast.Module(body=[node], type_ignores=[]), 'train.py', 'exec'), scope)
    state = dict(training_recipe_revision='recipe', reward_revision=prelaunch.SWEET_SPOT_REWARD,
                 training_distribution_revision=prelaunch.SWEET_SPOT_DISTRIBUTION,
                 policy_exploration_contract='bounded', placement_reward_identity=identity)
    def check(candidate):
        scope[node.name]({'hope_exact_resume_state':state}, SimpleNamespace(exploration_contract='bounded'),
                         'recipe', prelaunch.SWEET_SPOT_REWARD, prelaunch.SWEET_SPOT_DISTRIBUTION, candidate)
    check(identity)
    for key in ('reward_enabled', 'distance_scale_m', 'inside_bonus', 'reward_weight'):
        changed = deepcopy(identity)
        changed['reward'][key] = not identity['reward'][key] if key == 'reward_enabled' else identity['reward'][key]+.1
        with pytest.raises(ValueError, match='landing objective or sweet-spot reward'):
            check(changed)
    for key in ('objective_sha256', 'training_bank_sha256'):
        changed = deepcopy(identity); changed[key] = '0'*64
        with pytest.raises(ValueError, match='landing objective or sweet-spot reward'):
            check(changed)
    with pytest.raises(ValueError, match='landing objective or sweet-spot reward'):
        check(None)


@pytest.mark.parametrize('enabled', [True, False])
def test_export_records_switch_region_parameters_and_rejects_runtime_mismatch(sweet_bundle, enabled):
    task = resolved(sweet_bundle, f'task.sweet_spot.reward_enabled={str(enabled).lower()}')['task']
    env, _, _, motion, racket = export_fixture()
    racket.ready_release_enabled = False
    for key, value in task['motion'].items():
        setattr(motion, key, value)
    for key, value in task['racket'].items():
        attr = commands._RACKET_ALIASES.get(key, key)
        if key in commands._RACKET_ALIASES and key.endswith('range_per_clip'):
            value = commands._clip_ranges(value)
        setattr(racket, attr, value)
    metadata = exporter.sim2real_metadata(env, task)
    assert metadata['hitter_pure_sweet_spot_reward_enabled'] == str(enabled).lower()
    assert json.loads(metadata['hitter_pure_placement_reward_identity_json']) == exporter.placement_reward_identity(task)
    racket.sweet_spot_distance_scale_m += 1
    with pytest.raises(RuntimeError, match='sweet-spot reward'):
        exporter.sim2real_metadata(env, task)


def test_bank_labels_require_per_row_centre_and_inclusive_region(sweet_bundle):
    task = resolved(sweet_bundle)['task']
    receipt = json.loads(sweet_bundle[3].read_text())
    xr, yr = task['racket']['sweet_spot_x_range'], task['racket']['sweet_spot_y_range']
    center = np.array([np.mean(xr), np.mean(yr)], dtype=np.float32)
    arrays = {'intended_landing_xy':np.tile(center, (3,1)),
              'predicted_landing_xy':np.array([center,[xr[0],yr[0]],[xr[1],yr[1]]], dtype=np.float32)}
    robust.validate_sweet_spot_bank(arrays, receipt, xr, yr)
    wrong = {key:value.copy() for key,value in arrays.items()}
    wrong['intended_landing_xy'][1,0] += .01
    with pytest.raises(RuntimeError, match='canonical region centre'):
        robust.validate_sweet_spot_bank(wrong, receipt, xr, yr)
    wrong = {key:value.copy() for key,value in arrays.items()}
    wrong['predicted_landing_xy'][1,1] = yr[0] - .001
    with pytest.raises(RuntimeError, match='inclusive region'):
        robust.validate_sweet_spot_bank(wrong, receipt, xr, yr)


def test_sweet_spot_reward_revision_keeps_balanced_hold_optimizer_contract():
    scope = {}
    source = ROOT/'source/whole_body_tracking/whole_body_tracking/utils/balanced_hold_contract.py'
    exec(compile(source.read_text(), str(source), 'exec'), scope)
    validate = scope['validate_balanced_hold_algorithm']
    validate(prelaunch.SWEET_SPOT_REWARD, {'settled_aux_enabled':False})
    with pytest.raises(ValueError, match='ppo_balanced_hold'):
        validate(prelaunch.SWEET_SPOT_REWARD, {'settled_aux_enabled':True})
