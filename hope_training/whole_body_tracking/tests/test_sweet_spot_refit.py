"""Recipe 23 relabels targets without changing physical qualification or input evidence."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

pytest.importorskip('numba')
ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import refit_stationary_reach_bank as refitter
from ball_physics_profile import load_profile
from sweet_spot_profile import load_objective, objective_in_training_frame, objective_sha256

OBJECTIVE = REPO/'configs/return_placement_sweet_spot.yaml'
PHYSICS = REPO/'configs/ball_physics_ppq004_training.yaml'
SOURCE = ROOT/'assets/stationary_reach_v4_ppq004_generalization_20261022/train.npz'


def sample(count=128):
    with np.load(SOURCE, allow_pickle=False) as bank:
        ids = np.linspace(0,len(bank['clip'])-1,count,dtype=int)
        arrays = {key:bank[key][ids] if bank[key].ndim else bank[key] for key in bank.files}
    return arrays, json.loads(SOURCE.with_suffix('.receipt.json').read_text())


def objective_frame():
    return objective_in_training_frame(load_objective(OBJECTIVE), load_profile(PHYSICS))


def test_fixed_centre_preserves_original_targets_and_all_input_metadata():
    arrays, _ = sample(16)
    original = {key:value.copy() for key,value in arrays.items()}
    assert refitter.apply_landing_objective(arrays, None) is arrays
    selected = refitter.apply_landing_objective(arrays, objective_frame())
    np.testing.assert_array_equal(selected['intended_landing_xy'],
        np.broadcast_to(np.asarray([2.84,.2625], dtype=np.float32), (16,2)))
    np.testing.assert_array_equal(selected['input_intended_landing_xy'], original['intended_landing_xy'])
    for key,value in original.items():
        np.testing.assert_array_equal(arrays[key], value)
        if key != 'intended_landing_xy':
            np.testing.assert_array_equal(selected[key], value)
    accepted = np.arange(16)%3 != 0
    output = refitter.retained_rows(selected, accepted)
    np.testing.assert_array_equal(output['input_intended_landing_xy'], original['intended_landing_xy'][accepted])
    np.testing.assert_array_equal(output['input_source_row_index'], original['source_row_index'][accepted])
    np.testing.assert_array_equal(output['source_row_index'], np.flatnonzero(accepted))
    with pytest.raises(ValueError, match='landing-target chain'):
        refitter.apply_landing_objective(selected, objective_frame())


def test_stored_float32_full_model_return_hits_region_and_all_original_bounds_still_apply():
    arrays, receipt = sample()
    objective = objective_frame()
    a = refitter.apply_landing_objective(arrays, objective)
    x = a['target_position_offset'].astype(float)
    x[:,:2] += a['station_xy']
    incoming, spin, target = [a[key].astype(float) for key in
                              ('incoming_velocity','incoming_spin','intended_landing_xy')]
    p, offset = refitter.params_from_yaml(PHYSICS), refitter.table_offset_from_yaml(PHYSICS)
    velocity, normal, _, _, converged = refitter.retarget(x,incoming,spin,target,p,offset)
    a['target_velocity'], a['target_normal'] = velocity.astype(np.float32), normal.astype(np.float32)
    result, _, _ = refitter.check(x,incoming,spin,a['target_velocity'].astype(float),
                                 a['target_normal'].astype(float),target,p,offset)
    good, reasons = refitter.mask_valid(a,result,receipt,converged,landing_objective=objective)
    assert all((good & (a['clip']==side)).sum() > 10 for side in (0,1))
    assert np.all(result[good,8] < .003)
    general, _ = refitter.mask_valid(a,result,receipt,converged)
    assert np.all(~good | general)  # region adds a constraint; never loosens one
    i = np.flatnonzero(good)[0]
    # Inclusive edges and exclusion just outside are independent of fit distance.
    for x,y in ((objective['x_range'][0],objective['y_range'][0]),
                (objective['x_range'][1],objective['y_range'][1])):
        edge = result.copy()
        edge[i,:2], edge[i,8] = [x,y], 0.
        assert refitter.mask_valid(a,edge,receipt,converged,landing_objective=objective)[0][i]
    outside = result.copy()
    outside[i,0], outside[i,8] = objective['x_range'][0]-1e-6, 0.
    assert refitter.mask_valid(a,outside,receipt,converged)[0][i]
    assert not refitter.mask_valid(a,outside,receipt,converged,landing_objective=objective)[0][i]
    for column,value in ((2,.96),(3,0),(4,.50),(6,0),(7,7.01),(8,.004),(0,np.nan)):
        invalid = result.copy()
        invalid[i,column] = value
        assert not refitter.mask_valid(a,invalid,receipt,converged,landing_objective=objective)[0][i]


def make_source(directory, train_rows=100000):
    """Compressible source fixture; numerical inversion is tested separately above."""
    directory.mkdir()
    _, original = sample(2)
    for split,n,start in [('train',train_rows,0),('dev',4,200000),('test',4,300000)]:
        path = directory/f'{split}.npz'
        side = np.arange(n)%2
        np.savez_compressed(path, clip=side, station_xy=np.zeros((n,2),np.float32),
            target_position_offset=np.tile([1.,0.,1.],(n,1)).astype(np.float32),
            target_velocity=np.zeros((n,3),np.float32), target_normal=np.tile([1.,0.,0.],(n,1)).astype(np.float32),
            incoming_velocity=np.tile([-2.,0.,0.],(n,1)).astype(np.float32), incoming_spin=np.zeros((n,3),np.float32),
            intended_landing_xy=np.tile([2.5,.1],(n,1)).astype(np.float32),
            parent_id=np.arange(start,start+n), source_kind=side, generation_kind=side,
            source_row_index=np.arange(n), schema_version=np.array(1))
        receipt = dict(original, rows=n, bank_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        path.with_suffix('.receipt.json').write_text(json.dumps(receipt))


def fake_numerics(monkeypatch):
    def retarget(x,incoming,spin,target,params,offset):
        zeros = np.zeros_like(x)
        return zeros, zeros, zeros, zeros, np.ones(len(x),bool)
    def check(x,incoming,spin,racket,normal,target,params,offset):
        result = np.zeros((len(x),9))
        result[:,:2] = target
        return result,np.zeros_like(x),np.zeros_like(x)
    monkeypatch.setattr(refitter,'retarget',retarget)
    monkeypatch.setattr(refitter,'check',check)
    monkeypatch.setattr(refitter,'mask_valid',lambda a,*args,**kwargs:(np.ones(len(a['clip']),bool),{}))


@pytest.mark.parametrize('selected,revision',[(False,22),(True,23)])
def test_cli_writes_distinct_provenance_and_manifest_without_mutating_source(tmp_path,monkeypatch,selected,revision):
    source, output = tmp_path/'source',tmp_path/'output'
    make_source(source)
    original_sha = refitter.sha(source/'train.npz')
    fake_numerics(monkeypatch)
    argv = ['refitter','--source',str(source),'--output',str(output),'--physics',str(PHYSICS),'--threads','1']
    if selected:
        argv.extend(['--landing-objective',str(OBJECTIVE)])
    monkeypatch.setattr(sys,'argv',argv)
    refitter.main()
    assert refitter.sha(source/'train.npz') == original_sha
    manifest = yaml.safe_load((output/'training_overrides.yaml').read_text())
    assert manifest['task']['racket']['venue_tuple_bank_recipe_revision'] == revision
    for split in ('train','dev','test'):
        receipt = json.loads((output/f'{split}.receipt.json').read_text())
        assert receipt['contract']['recipe_revision'] == revision
        assert receipt['contract']['qualification'] == 'full_ppq004_first_bounce_v1'
        assert receipt['contract']['distribution'] == 'stationary_reach_v1'
        assert receipt['bank_sha256'] == refitter.sha(output/f'{split}.npz')
        with np.load(output/f'{split}.npz') as result, np.load(source/f'{split}.npz') as original:
            for key in ('incoming_spin','incoming_velocity','generation_kind','parent_id','target_position_offset'):
                np.testing.assert_array_equal(result[key],original[key])
            if selected:
                np.testing.assert_array_equal(result['input_intended_landing_xy'],original['intended_landing_xy'])
                np.testing.assert_allclose(result['intended_landing_xy'],np.tile([2.84,.2625],(len(result['clip']),1)),atol=1e-7)
            else:
                assert 'input_intended_landing_xy' not in result
                np.testing.assert_array_equal(result['intended_landing_xy'],original['intended_landing_xy'])
        if selected:
            block = manifest['task']['landing_objective']
            assert block == dict(path=str(OBJECTIVE),sha256=objective_sha256(OBJECTIVE),descriptor=load_objective(OBJECTIVE))
            assert receipt['source']['landing_objective_yaml_sha256'] == block['sha256']
            assert receipt['contract']['landing_objective_sha256'] == block['sha256']
            assert receipt['contract']['landing_objective'] == block['descriptor']
            assert receipt['contract']['training_distribution_revision'] == 'stationary_reach_ppq004_sweet_spot_v1'
        else:
            assert 'landing_objective' not in manifest['task']
            assert 'landing_objective' not in receipt['contract']
            assert 'landing_objective_yaml_sha256' not in receipt['source']


def test_sweet_spot_refit_keeps_original_minimum_rows_per_side(tmp_path,monkeypatch):
    source, output = tmp_path/'source',tmp_path/'output'
    make_source(source,train_rows=99998)
    fake_numerics(monkeypatch)
    monkeypatch.setattr(sys,'argv',['refitter','--source',str(source),'--output',str(output),
        '--physics',str(PHYSICS),'--landing-objective',str(OBJECTIVE),'--threads','1'])
    with pytest.raises(RuntimeError,match='minimum 50000 train rows per side'):
        refitter.main()
    assert not (output/'training_overrides.yaml').exists()
    assert not (output/'train.npz').exists()


def test_changed_objective_during_generation_prevents_bank_publication(tmp_path,monkeypatch):
    source, output = tmp_path/'source',tmp_path/'output'
    make_source(source)
    objective = tmp_path/'objective.yaml'
    objective.write_bytes(OBJECTIVE.read_bytes())
    fake_numerics(monkeypatch)
    original_retarget = refitter.retarget
    def changing_retarget(*args):
        result = original_retarget(*args)
        objective.write_text(objective.read_text()+'# changed during generation\n')
        return result
    monkeypatch.setattr(refitter,'retarget',changing_retarget)
    monkeypatch.setattr(sys,'argv',['refitter','--source',str(source),'--output',str(output),
        '--physics',str(PHYSICS),'--landing-objective',str(objective),'--threads','1'])
    with pytest.raises(RuntimeError,match='objective sources changed'):
        refitter.main()
    assert not (output/'training_overrides.yaml').exists()
    assert not (output/'train.npz').exists()
