"""Explicit preflight/scored playback; never substitute another phase or policy."""
import argparse,json,shutil,subprocess
from pathlib import Path


def select_recording(session,phase,attempt=None):
    session=Path(session).resolve()
    if phase not in ('preflight','scored'):raise ValueError('Explicit recording phase required')
    if attempt is not None:
        if Path(attempt).name!=attempt or not attempt.startswith('attempt-'):raise ValueError('Expected an attempt directory name')
        directory=session/attempt/phase
    else:
        pointer=session/('latest-recorded-preflight.txt' if phase=='preflight' else 'latest-recorded-diagnostic.txt')
        if not pointer.exists():raise FileNotFoundError(f'No {phase} recording pointer; specify an attempt. No other-phase fallback.')
        directory=Path(pointer.read_text().strip()).resolve()
    directory=directory.resolve()
    if directory.parent.parent!=session or directory.name!=phase:raise ValueError('Recording pointer escaped the requested session/phase')
    summary=json.loads((directory/'summary.json').read_text())
    snapshot=json.loads((session/'snapshot.json').read_text());manifest=json.loads((session/'bundle/manifest.json').read_text())
    receipt=json.loads((directory/'video-receipt.json').read_text())
    expected='preflight' if phase=='preflight' else '100-serve'
    if summary['phase']!=expected:raise ValueError('Recording summary phase mismatch')
    if summary['iteration']!=snapshot['iteration'] or summary['checkpoint_sha256']!=snapshot['sha256']:raise ValueError('Recording checkpoint mismatch')
    if summary['onnx_sha256']!=manifest['onnx_sha256'] or receipt['onnx_sha256']!=manifest['onnx_sha256']:raise ValueError('Recording ONNX mismatch')
    movie=directory/'diagnostic.mp4'
    if not movie.is_file() or movie.stat().st_size==0:raise FileNotFoundError(f'No {phase} video at {movie}; no fallback')
    return movie,summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',type=Path,required=True);p.add_argument('--phase',choices=['preflight','scored'],required=True);p.add_argument('--attempt');p.add_argument('--check-only',action='store_true');a=p.parse_args()
    movie,summary=select_recording(a.session,a.phase,a.attempt)
    print(f'Replay {a.phase} | checkpoint model_{summary["iteration"]} | bundle {a.session}/bundle | {movie}',flush=True)
    if a.check_only:return
    player=shutil.which('ffplay')
    if player:raise SystemExit(subprocess.call([player,'-autoexit',str(movie)]))
    opener=shutil.which('xdg-open')
    if opener:raise SystemExit(subprocess.call([opener,str(movie)]))
    raise SystemExit('No installed desktop video opener: '+str(movie))

if __name__=='__main__':main()
