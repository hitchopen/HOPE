"""Publication contract checks only; no MuJoCo/ROS or behavioral trials."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'publish_frozen_gru_exports.py'
spec = importlib.util.spec_from_file_location('frozen_publication', SCRIPT)
pub = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pub)


def test_relabelled_actor_evidence_is_rejected():
    base = pub.TEMPLATE / 'policy/exported'
    manifest = pub.load(base / 'manifest.json')
    proof = pub.load(base / 'actor-load-verification.json')
    wrong = {'iteration':13000, 'sha256':manifest['checkpoint_sha256'], 'run_id':pub.RUN}
    with pytest.raises(AssertionError):
        pub.validate_identity(manifest, proof, wrong)


def test_standalone_verifier_uses_its_own_actor_and_fixtures(tmp_path):
    # Existing model19000 is a bounded packaging fixture, not a new evaluation.
    source = pub.TEMPLATE / 'policy/exported'
    for name in ['runtime','manifest.json','actor.onnx','parity_vectors.npz']:
        if (source / name).is_dir():
            shutil.copytree(source / name, tmp_path / name, ignore=shutil.ignore_patterns('__pycache__'))
        else:
            shutil.copyfile(source / name, tmp_path / name)
    (tmp_path / 'verify.py').write_text(pub.VERIFY)
    result = subprocess.run([sys.executable,str(tmp_path/'verify.py')],cwd=tmp_path,check=True,capture_output=True,text=True)
    report = json.loads(result.stdout)
    assert report['iteration'] == 19000
    assert report['sequence_steps'] == 64 and report['zero_reset_repeat'] == 'PASS'
    assert report['onnx_sha256'] == pub.sha(source / 'actor.onnx')
    assert report['behavioral_evaluation_performed'] is False


def test_package_and_extract_using_explicit_existing_actor_fixture(tmp_path, monkeypatch):
    # Exercise packaging without pretending that the four new actors were exported.
    original = pub.TEMPLATE / 'policy/exported'
    fixture = tmp_path / 'fixture'
    fixture.mkdir()
    for name in ['manifest.json','actor.onnx','actor-load-verification.json','parity_vectors.npz']:
        shutil.copyfile(original / name, fixture / name)
    (fixture / 'export-environment.json').write_text(json.dumps({'scope':'existing model19000 packaging fixture only; no fresh export claimed'}))
    manifest = pub.load(fixture / 'manifest.json')
    params = pub.load(pub.TEMPLATE / 'policy/params/training-config-provenance.json')
    spec = {'iteration':19000,'sha256':manifest['checkpoint_sha256'],'run_id':pub.RUN,'params_sha256':params['original_training_config_sha256']}
    model_root = tmp_path / 'temporary-models'
    model_root.mkdir()
    session = tmp_path / 'session'
    session.mkdir()
    monkeypatch.setattr(pub, 'MODELS', model_root)
    packaged = pub.package_one(session,spec,fixture,'packaging-test-fixture-only')
    verified = pub.verify_package(packaged,spec)
    assert verified['checkpoint_iteration'] == 19000
    assert pub.sha(packaged/'policy/exported/actor.onnx') == pub.sha(original/'actor.onnx')
    assert not list(packaged.rglob('*.pt'))
    assert not list(packaged.rglob('__pycache__'))
