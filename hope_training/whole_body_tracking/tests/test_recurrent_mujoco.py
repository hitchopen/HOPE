"""Check an explicitly selected recurrent bundle, including the published model29999 actor."""
from pathlib import Path
import json
import os
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'a3_deploy/a3_deploy_example/reference'))
from a3_deploy_onnx_ref_pingpong.recurrent_policy import RecurrentOnnxPolicy


@pytest.fixture
def bundle():
    path=os.environ.get('HOPE_GRU_EVAL_BUNDLE')
    if not path:pytest.skip('set HOPE_GRU_EVAL_BUNDLE to an exported checkpoint bundle')
    return Path(path)


def test_memory_is_carried_and_new_episode_matches_fresh_policy(bundle):
    fixture=np.load(bundle/'parity_vectors.npz')
    policy=RecurrentOnnxPolicy(bundle)
    obs=fixture['obs'][0]
    first=policy.infer(obs)
    first_hidden=policy.hidden.copy()
    for row in fixture['obs'][1:8]:policy.infer(row)
    assert np.max(np.abs(policy.hidden-first_hidden))>1e-3
    assert policy.reset_count==0
    policy.reset_episode()
    assert not policy.hidden.any()
    np.testing.assert_array_equal(policy.infer(obs),first)
    np.testing.assert_array_equal(policy.hidden,first_hidden)
    assert policy.reset_count==1


def test_nonfinite_observation_fails_before_mutating_memory(bundle):
    policy=RecurrentOnnxPolicy(bundle)
    before=policy.hidden.copy()
    with pytest.raises(FloatingPointError):policy.infer(np.full(110,np.nan))
    np.testing.assert_array_equal(policy.hidden,before)


def test_corrupted_bundle_hash_rejected(bundle,tmp_path):
    doc=json.loads((bundle/'manifest.json').read_text())
    doc['onnx_sha256']='0'*64
    (tmp_path/'manifest.json').write_text(json.dumps(doc))
    (tmp_path/'actor.onnx').symlink_to(bundle/'actor.onnx')
    with pytest.raises(ValueError,match='SHA'):RecurrentOnnxPolicy(tmp_path)


def test_reference_marker_stays_at_supplied_strike_point_without_mutating_plant(bundle):
    from contextlib import nullcontext
    from types import SimpleNamespace
    import importlib.util
    import mujoco
    script=ROOT/'hope_training/whole_body_tracking/scripts/mujoco_eval_recurrent.py'
    spec=importlib.util.spec_from_file_location('recurrent_marker_test',script)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    plant=module.Plant(json.loads((bundle/'manifest.json').read_text()))
    overlays=[]
    scene=mujoco.MjvScene(plant.model,maxgeom=16)
    plant.viewer=SimpleNamespace(user_scn=scene,lock=nullcontext,sync=lambda:None,
                                 set_texts=overlays.append)
    qpos=plant.data.qpos.copy()
    previous_action=plant.previous_action.copy()
    point=np.array([.58,-.44,1.1])
    for clock in [.6,-.2]:
        target=module.ObsTarget(point.copy(),np.array([2.,.2,1.]),clock)
        plant.draw(target,0)
        np.testing.assert_allclose(scene.geoms[0].pos,point,rtol=0,atol=1e-7)
        assert scene.geoms[0].label=='REFERENCE STRIKE TARGET'
        assert scene.geoms[0].size[0]==pytest.approx(.04)
        assert 'REFERENCE STRIKE TARGET ONLY' in overlays[-1][2]
        np.testing.assert_array_equal(target.pos_w,point)
        np.testing.assert_array_equal(plant.data.qpos,qpos)
        np.testing.assert_array_equal(plant.previous_action,previous_action)
