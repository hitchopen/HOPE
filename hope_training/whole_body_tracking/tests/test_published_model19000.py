"""Ensure the additional package loads its own actor and rejects the final actor."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / 'a3_deploy/a3_deploy_example/models/model_19000'


def test_extracted_package_sequence_and_reset(tmp_path):
    with zipfile.ZipFile(PACKAGE / 'model_19000_gru_bundle.zip') as archive:
        archive.extractall(tmp_path)
    manifest = json.loads((tmp_path / 'policy/exported/manifest.json').read_text())
    assert manifest['checkpoint_sha256'] == '744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0'
    for name, evidence in json.loads((tmp_path / 'package-manifest.json').read_text())['files'].items():
        assert hashlib.sha256((tmp_path / name).read_bytes()).hexdigest() == evidence['sha256']
    result = subprocess.run([sys.executable, str(tmp_path / 'policy/exported/verify.py')], cwd=tmp_path, check=True, capture_output=True, text=True)
    report = json.loads(result.stdout)
    assert report['iteration'] == 19000 and report['sequence_steps'] == 64
    assert report['zero_reset_repeat'] == 'PASS'


def test_model19000_launcher_rejects_model29999(tmp_path):
    result = subprocess.run([sys.executable, str(ROOT / 'hope_training/whole_body_tracking/scripts/run_model19000_mujoco.py'), '--bundle', str(PACKAGE.with_name('model_29999') / 'policy/exported'), '--verify-only', '--output-root', str(tmp_path)], capture_output=True, text=True)
    assert result.returncode != 0
    assert 'Expected published model19000, no fallback' in result.stderr
    assert not list(tmp_path.iterdir())
