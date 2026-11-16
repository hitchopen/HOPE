import sys,json
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from replay_gru_recording import select_recording

@pytest.fixture
def session(tmp_path):
    (tmp_path/'bundle').mkdir();(tmp_path/'bundle/manifest.json').write_text(json.dumps({'onnx_sha256':'actor'}))
    (tmp_path/'snapshot.json').write_text(json.dumps({'iteration':29500,'sha256':'checkpoint'}))
    for phase in ['preflight','scored']:
        p=tmp_path/'attempt-example'/phase;p.mkdir(parents=True)
        (p/'summary.json').write_text(json.dumps({'phase':'preflight' if phase=='preflight' else '100-serve','iteration':29500,'checkpoint_sha256':'checkpoint','onnx_sha256':'actor'}))
        (p/'video-receipt.json').write_text(json.dumps({'onnx_sha256':'actor'}));(p/'diagnostic.mp4').write_bytes(b'test')
    return tmp_path


def test_explicit_preflight_selection_never_uses_scored(session):
    movie,_=select_recording(session,'preflight','attempt-example');assert movie.parent.name=='preflight'
    movie.unlink()
    with pytest.raises(FileNotFoundError):select_recording(session,'preflight','attempt-example')


def test_missing_preflight_pointer_does_not_fall_back_to_scored(session):
    (session/'latest-recorded-diagnostic.txt').write_text(str(session/'attempt-example/scored'))
    with pytest.raises(FileNotFoundError):select_recording(session,'preflight')
    (session/'latest-recorded-preflight.txt').write_text(str(session/'attempt-example/scored'))
    with pytest.raises(ValueError):select_recording(session,'preflight')


def test_identity_and_path_fail_closed(session):
    with pytest.raises(ValueError):select_recording(session,'preflight','../attempt-example')
    p=session/'attempt-example/preflight/summary.json';x=json.loads(p.read_text());x['iteration']=5300;p.write_text(json.dumps(x))
    with pytest.raises(ValueError):select_recording(session,'preflight','attempt-example')
