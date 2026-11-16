"""Publish four exact frozen GRU exports; no simulator, training, ROS or video runs.

Requires the existing grasping container and a host Python with NumPy/ORT.
An export is never replaced by another iteration or marked verified on failure.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
import zipfile

ROOT = Path(__file__).resolve().parents[3]
ITERATIONS = (13000, 14000, 14600, 20000)
BRANCH = 'feature/stationary-gru-mujoco-eval'
ORIGIN = 'https://github.com/1786089174zth-cell/HOPE.git'
RUN = 'gru-formal-2b8c09938a'
SOURCE = '33c609022efbb4e85fe6a58a5a8947b0fbd46c76'
MODELS = ROOT / 'a3_deploy/a3_deploy_example/models'
TEMPLATE = MODELS / 'model_19000'
SCRIPTS = ROOT / 'hope_training/whole_body_tracking/scripts'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def git(*args, check=True):
    return subprocess.run(['git', '-C', str(ROOT), *args], check=check, capture_output=True, text=True)


def repository():
    assert git('branch', '--show-current').stdout.strip() == BRANCH, 'Wrong feature branch'
    assert git('remote', 'get-url', 'origin').stdout.strip() == ORIGIN
    assert git('remote', 'get-url', '--push', 'origin').stdout.strip() == ORIGIN
    return git('rev-parse', 'HEAD').stdout.strip()


def remote_inspection(session):
    result = git('fetch', 'origin', check=False)
    (session / 'fetch.txt').write_text(result.stdout + result.stderr)
    if result.returncode:
        print('FETCH BLOCKED; continuing independent export/package work:', result.stderr.strip(), flush=True)
        return False
    ref = 'refs/remotes/origin/' + BRANCH
    if git('show-ref', '--verify', '--quiet', ref, check=False).returncode == 0:
        print('Local/remote divergence:', git('rev-list', '--left-right', '--count', 'HEAD...' + ref).stdout.strip(), flush=True)
        print(git('log', '--oneline', '--left-right', 'HEAD...' + ref).stdout, flush=True)
        print('Changed remote paths:', git('diff', '--name-only', 'HEAD...' + ref).stdout, flush=True)
        if git('merge-base', '--is-ancestor', ref, 'HEAD', check=False).returncode:
            raise RuntimeError('Remote has changes absent locally. Inspect/integrate on this same branch before publication; no merge/reset performed.')
    return True


def stage(name, command, log, timeout=240):
    started = time.monotonic()
    print('START', name, flush=True)
    with Path(log).open('a') as stream:
        stream.write('COMMAND ' + json.dumps([str(x) for x in command]) + '\n')
        stream.flush()
        result = subprocess.run([str(x) for x in command], stdout=stream, stderr=subprocess.STDOUT, timeout=timeout)
    print(f'END {name}: rc={result.returncode}, {time.monotonic()-started:.2f}s | {log}', flush=True)
    if result.returncode:
        raise RuntimeError(name + ' failed; no fallback: ' + str(log))


def validate_snapshot(spec):
    assert spec['iteration'] in ITERATIONS and spec['run_id'] == RUN
    assert spec['source_commit'] == SOURCE
    assert sha(spec['snapshot']) == spec['sha256']
    assert sha(spec['source']) == spec['sha256']
    for name, digest in spec['params_sha256'].items():
        assert sha(Path(spec['snapshot']).parent / 'params' / name) == digest


def validate_identity(manifest, proof, spec):
    assert manifest['checkpoint_iteration'] == proof['iteration'] == spec['iteration']
    assert manifest['checkpoint_sha256'] == proof['checkpoint_sha256'] == spec['sha256']
    assert manifest['wandb_run_id'] == spec['run_id'] == RUN
    assert manifest['onnx_sha256'] == proof['onnx_sha256']
    assert proof['result'] == 'PASS' and proof['gru_weights_and_bias_match'] and proof['finite_actor_weights']


# Runs outside this repository too; parity fixtures come from this exact checkpoint.
VERIFY = '''"""Standalone exact-actor64-step parity and recurrent reset verification."""
from pathlib import Path
import json
import numpy as np
from runtime import RecurrentOnnxPolicy
root=Path(__file__).resolve().parent
p=RecurrentOnnxPolicy(root);f=np.load(root/'parity_vectors.npz');p.hidden=f['initial_h'].copy()
assert len(f['obs'])==64 and np.any(p.hidden) and int(f['reset_at'])==32
aerr=herr=0.
for i,obs in enumerate(f['obs']):
 if i==int(f['reset_at']):p.reset_episode()
 action=p.infer(obs)
 aerr=max(aerr,float(np.max(abs(action-f['actions'][i,0]))));herr=max(herr,float(np.max(abs(p.hidden-f['hidden'][i]))))
 np.testing.assert_allclose(action,f['actions'][i,0],rtol=2e-4,atol=2e-5)
 np.testing.assert_allclose(p.hidden,f['hidden'][i],rtol=2e-4,atol=2e-5)
p.reset_episode();a=p.infer(f['obs'][0]);h=p.hidden.copy()
for obs in f['obs'][1:17]:p.infer(obs)
p.reset_episode();np.testing.assert_array_equal(a,p.infer(f['obs'][0]));np.testing.assert_array_equal(h,p.hidden)
print(json.dumps({'result':'PASS','iteration':p.manifest['checkpoint_iteration'],'checkpoint_sha256':p.manifest['checkpoint_sha256'],'onnx_sha256':p.manifest['onnx_sha256'],'sequence_steps':64,'reset_at':32,'action_max_abs_error':aerr,'hidden_max_abs_error':herr,'atol':2e-5,'rtol':2e-4,'zero_reset_repeat':'PASS','behavioral_evaluation_performed':False}))
'''


def export_one(session, spec):
    model = Path(spec['snapshot']).parent
    bundle = model / 'bundle'
    receipt = model / 'export-verified.json'
    if receipt.exists():
        record = load(receipt)
        assert record['checkpoint_sha256'] == spec['sha256']
        for name, digest in record['files'].items():
            assert sha(bundle / name) == digest, 'Changed verified export; do not overwrite'
        validate_identity(load(bundle / 'manifest.json'), load(bundle / 'actor-load-verification.json'), spec)
        print('REUSE verified export', spec['iteration'], flush=True)
        return bundle
    # Never trigger Distrobox's interactive create-container fallback.
    stage('existing grasping container access', ['podman', 'inspect', '--type', 'container', 'grasping', '--format', '{{.Id}}'], model / 'container.log', 30)
    candidate = Path(tempfile.mkdtemp(prefix='export-attempt-', dir=model))
    shell = candidate / 'export.sh'
    command = 'set -euo pipefail\n'
    command += 'if [[ -n "${CONDA_PREFIX:-}" ]]; then conda deactivate; fi\n'
    command += 'cd ' + shlex.quote(str(ROOT / 'hope_training/whole_body_tracking')) + '\nsource setup_train_env.sh\n'
    command += "export CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1\n"
    commands = [
        ['scripts/export_recurrent_mujoco.py', '--checkpoint', spec['snapshot'], '--expected-sha256', spec['sha256'], '--output-dir', candidate],
        ['scripts/verify_recurrent_bundle_weights.py', '--checkpoint', spec['snapshot'], '--sha256', spec['sha256'], '--iteration', spec['iteration'], '--bundle', candidate, '--baseline', TEMPLATE / 'policy/exported/manifest.json', '--report', candidate / 'actor-load-verification.json'],
    ]
    for args in commands:
        command += 'hope_isaac_py ' + shlex.join([str(x) for x in args]) + '\n'
    versions = "import json,sys,importlib.metadata as m; from pathlib import Path; Path(" + repr(str(candidate / 'export-environment.json')) + ").write_text(json.dumps({'python':sys.version,'packages':{k:m.version(k) for k in ['torch','onnx','tensordict','numpy']}}))"
    command += 'hope_isaac_py -c ' + shlex.quote(versions) + '\n'
    shell.write_text(command)
    stage('CPU exact actor export/initializer verification model_' + str(spec['iteration']), ['distrobox', 'enter', 'grasping', '--', 'bash', shell], model / 'export.log')
    manifest = load(candidate / 'manifest.json');proof = load(candidate / 'actor-load-verification.json')
    validate_identity(manifest, proof, spec)
    assert sha(candidate / 'actor.onnx') == manifest['onnx_sha256']
    # Parity/reset are performed during packaging before any publication receipt.
    if bundle.exists():
        raise FileExistsError('Unverified bundle already exists; preserved at ' + str(bundle))
    candidate.rename(bundle)
    save(receipt, {'checkpoint_sha256':spec['sha256'], 'files':{name:sha(bundle / name) for name in ['actor.onnx','manifest.json','actor-load-verification.json','parity_vectors.npz','export-environment.json']}, 'scope':'PyTorch sequential oracle and exact loaded initializer verification; host ONNX parity/reset still required before packaging'})
    return bundle


def readme(spec, manifest, proof):
    n = spec['iteration']
    return f'''# model_{n}: recurrent StationaryRally actor

**export verified; no new behavioral evaluation performed.**

[Download portable ZIP](model_{n}_gru_bundle.zip) · [ONNX](policy/exported/actor.onnx) · [Manifest](policy/exported/manifest.json) · [Runtime configuration](policy/params/runtime.json) · [Model index](../README.md)

Run `{RUN}`; exact iteration **{n}**, no substitution. Training source `{SOURCE}`.

- Checkpoint SHA256: `{spec['sha256']}`.
- ONNX SHA256: `{manifest['onnx_sha256']}`.
- Actor parameter fingerprint: `{proof['actor_parameter_sha256']}`.

Original large PT and full private training YAML remain in the local snapshot; no PT is distributed. This package contains its own actual ONNX, saved PyTorch sequence fixtures, exact initializer-verification evidence, host parity/reset report, normalization/joint/action/actuator/control configuration, standalone runtime, pinned dependency lists and per-file hashes. No evidence from another checkpoint is relabelled.

## Verify and load

Use an existing compatible Python with the versions in `policy/exported/requirements-runtime.txt` (NumPy/ONNX Runtime); no automatic installation is performed. From this repository's root:

```bash
python a3_deploy/a3_deploy_example/models/model_{n}/policy/exported/verify.py
```

After extracting the ZIP, from the extracted root:

```bash
python policy/exported/verify.py
cd policy/exported
python - <<'PYTHON'
from pathlib import Path
from runtime import RecurrentOnnxPolicy
actor = RecurrentOnnxPolicy(Path('.'))
actor.reset_episode()
print(actor.manifest['checkpoint_iteration'], actor.hidden.shape)
# Each real control tick: action = actor.infer(observation_110)
# The observation must follow this bundle's saved contract; carry memory.
PYTHON
```

The standalone test checks64 consecutive saved PyTorch/ONNX outputs with arbitrary nonzero initial memory, a zero reset at step32, and exact repeated inference after resetting from different histories. `host-verification.json` records observed errors; the manifest records the exporter PyTorch stateful-oracle error. Export verification is not standing/ball/robot performance evidence.

## Recurrent runtime contract

Inputs: `obs:float32[1,110]`, `h_in:float32[1,1,110]`; outputs: `actions:float32[1,31]`, `h_out:float32[1,1,110]`. Initialize hidden state to zero once at genuine episode start; carry the returned hidden state through all control ticks/serves/recovery. Reset only for a real new episode. Never reuse training-batch memory or reset every tick/ball. Normalization is `{manifest['normalization']}` and owned by the graph. No new actor inputs or side labels are added.

The manifest/runtime JSON define all observation slices, saved joint order, passive columns, affine action scale/offset/clamp, delay FIFO, actuator parameters and20ms control timing. Original training metadata is4×5ms; the compatible existing physical MuJoCo evaluator uses the documented20×1ms override. Actor inference itself needs no MuJoCo, ROS or training libraries. Native feed-forward/hardware runners are incompatible with this hidden-state ABI and must not be pointed at this actor without an adapter.

Shared nominal MuJoCo/planner configuration is included only for compatibility, not as evidence of a new evaluation. No new standing trials, ball evaluations, comparisons or videos were run. Do not infer performance from model19000/29999 or prior results. Remaining PhysX/MuJoCo sensing, drive/contact, lifecycle and scoring differences remain documented in the existing evaluation docs. No real-robot deployment claim is made.
'''


def verify_package(path, spec):
    record = load(path / 'package-manifest.json')
    assert record['checkpoint_iteration'] == spec['iteration'] and record['checkpoint_sha256'] == spec['sha256']
    for name, metadata in record['files'].items():
        assert sha(path / name) == metadata['sha256'], name
    manifest = load(path / 'policy/exported/manifest.json');proof = load(path / 'policy/exported/actor-load-verification.json')
    validate_identity(manifest, proof, spec)
    assert sha(path / 'policy/exported/actor.onnx') == manifest['onnx_sha256']
    report = load(path / 'policy/exported/host-verification.json')
    assert report['iteration'] == spec['iteration'] and report['checkpoint_sha256'] == spec['sha256']
    assert report['result'] == report['zero_reset_repeat'] == 'PASS' and report['sequence_steps'] == 64
    archive = path / f'model_{spec["iteration"]}_gru_bundle.zip'
    assert sha(archive) == (path / (archive.name + '.sha256')).read_text().split()[0]
    assert '/home/' not in (path / 'policy/exported/manifest.json').read_text()
    return manifest


def package_one(session, spec, bundle, code_commit):
    n = spec['iteration'];destination = MODELS / f'model_{n}'
    if destination.exists():
        verify_package(destination, spec)
        print('REUSE verified package', n, flush=True)
        return destination
    candidate = Path(tempfile.mkdtemp(prefix=f'package-{n}-', dir=session))
    exported = candidate / 'policy/exported';exported.mkdir(parents=True)
    # A companion deploy.yaml binds model19000 specifically; never clone it into another actor.
    params = candidate / 'policy/params';shutil.copytree(TEMPLATE / 'policy/params', params, ignore=shutil.ignore_patterns('deploy.yaml'))
    for name in ['runtime', 'requirements-runtime.txt', 'requirements-mujoco.txt']:
        source = TEMPLATE / 'policy/exported' / name
        if source.is_dir():shutil.copytree(source, exported / name, ignore=shutil.ignore_patterns('__pycache__'))
        else:shutil.copyfile(source, exported / name)
    for name in ['actor.onnx', 'actor-load-verification.json', 'parity_vectors.npz', 'export-environment.json']:
        shutil.copyfile(bundle / name, exported / name)
    manifest = load(bundle / 'manifest.json');proof = load(bundle / 'actor-load-verification.json')
    validate_identity(manifest, proof, spec)
    assert sha(exported / 'actor.onnx') == manifest['onnx_sha256']
    baseline = load(TEMPLATE / 'policy/exported/manifest.json')
    for name in ['normalization','normalization_keys','joint_names','observation_layout','hidden_shape','action','actuator','root_default','physics_dt','decimation','control_dt','task','motion_cfg','visual_table','racket_site']:
        assert manifest[name] == baseline[name], 'Shared portable runtime contract mismatch: ' + name
    provenance = load(params / 'training-config-provenance.json')
    assert spec['params_sha256'] == provenance['original_training_config_sha256']
    manifest['checkpoint'] = f'model_{n}.pt';manifest['checkpoint_distributed'] = False
    manifest['publication'] = {'training_source_commit':SOURCE,'code_commit':code_commit,'exported_manifest_sha256':sha(bundle / 'manifest.json'),'weights_changed':False,'runtime_environment_observed':{'python':sys.version,'packages':{k:importlib.metadata.version(k) for k in ['numpy','onnxruntime']}},'behavioral_evaluation_performed':False,'verification_scope':'exact actor initializers;64-step PyTorch/ONNX parity; recurrent reset repeat','configuration_sha256':{str(p.relative_to(candidate)):sha(p) for p in params.iterdir()},'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__), SCRIPTS/'export_recurrent_mujoco.py',SCRIPTS/'verify_recurrent_bundle_weights.py',SCRIPTS/'portable_model_config.py']}}
    save(exported / 'manifest.json', manifest)
    (exported / 'verify.py').write_text(VERIFY)
    result = subprocess.run([sys.executable, str(exported / 'verify.py')], cwd=session, check=True, capture_output=True, text=True, timeout=60)
    report = json.loads(result.stdout);assert report['iteration'] == n
    save(exported / 'host-verification.json', report)
    (candidate / 'README.md').write_text(readme(spec,manifest,proof))
    # Bytecode belongs to the local interpreter, not a deployment artifact.
    for cache in candidate.rglob('__pycache__'):shutil.rmtree(cache)
    files = {str(p.relative_to(candidate)):{'sha256':sha(p),'bytes':p.stat().st_size} for p in sorted(candidate.rglob('*')) if p.is_file()}
    for name, metadata in files.items():
        assert metadata['bytes'] < 100*1024*1024 and not name.endswith('.pt')
    save(candidate / 'package-manifest.json', {'run_id':RUN,'checkpoint_iteration':n,'checkpoint_sha256':spec['sha256'],'onnx_sha256':manifest['onnx_sha256'],'publication_code_commit':code_commit,'status':'export verified; no new behavioral evaluation performed','files':files})
    (candidate / 'SHA256SUMS').write_text(''.join(f'{entry["sha256"]}  {name}\n' for name,entry in files.items()))
    archive = candidate / f'model_{n}_gru_bundle.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
        for p in sorted(candidate.rglob('*')):
            if p.is_file() and p != archive:z.write(p,p.relative_to(candidate))
    (candidate / (archive.name + '.sha256')).write_text(sha(archive) + '  ' + archive.name + '\n')
    verify_package(candidate, spec)
    # Verify actual distributed ZIP from an unrelated directory.
    with tempfile.TemporaryDirectory(prefix='verify-extracted-',dir=session) as temporary:
        with zipfile.ZipFile(archive) as z:z.extractall(temporary)
        subprocess.run([sys.executable,str(Path(temporary)/'policy/exported/verify.py')],cwd=temporary,check=True,timeout=60)
    candidate.rename(destination)
    print('PACKAGE VERIFIED', destination, report, flush=True)
    return destination


def update_docs(plan, packages):
    table='\n## Additional frozen recurrent exports\n\n**export verified; no new behavioral evaluation performed.** Run `'+RUN+'`. These bundles have no new standing/ball results or videos.\n\n| Iteration | Portable bundle | Loading and verification |\n|---|---|---|\n'
    for spec in plan['models']:
        n=spec['iteration'];table+=f'| {n} | [ZIP](a3_deploy/a3_deploy_example/models/model_{n}/model_{n}_gru_bundle.zip) | [README](a3_deploy/a3_deploy_example/models/model_{n}/README.md) |\n'
    marker='<!-- four-frozen-gru-exports -->'
    for path, section in [(ROOT/'README.md',table),(MODELS/'README.md',table.replace('a3_deploy/a3_deploy_example/models/',''))]:
        text=path.read_text()
        if marker in text:text=text[:text.index(marker)].rstrip()+'\n'
        path.write_text(text.rstrip()+'\n\n'+marker+'\n'+section)
    log=ROOT/'docs/worklog/2026-11-07-four-checkpoint-export-publication.md'
    text='# 2026-11-07 — four exact recurrent export bundles\n\nExisting branch `'+BRANCH+'`; origin `'+ORIGIN+'`; source `'+SOURCE+'`; run `'+RUN+'`.\n\n**export verified; no new behavioral evaluation performed.** No trials, ball evaluations, training, comparisons or video generation were run. Original model19000/29999 packages, existing results/videos and training checkout remain unchanged. Large PT and private full YAML retained locally.\n\n'
    for spec,path in zip(plan['models'],packages):
        manifest=load(path/'policy/exported/manifest.json');report=load(path/'policy/exported/host-verification.json');archive=path/f'model_{spec["iteration"]}_gru_bundle.zip'
        text+=f'- model_{spec["iteration"]}: checkpoint `{spec["sha256"]}`; ONNX `{manifest["onnx_sha256"]}`; ZIP `{sha(archive)}` ({archive.stat().st_size}bytes). Exact PyTorch actor/GRU initializer match PASS;64-step sequential action max error {report["action_max_abs_error"]:.9g}, hidden {report["hidden_max_abs_error"]:.9g}; zero-reset repeat PASS. Extracted standalone ZIP check PASS.\n'
    text+='\nProvenance/software/observation/action/actuator/control contracts and per-file configuration/source hashes are included in each package. Exporter code commit precedes the packaging commit; no behavioral evidence is relabelled. Only this four-export publisher, the four new model directories, shared README/model index and dated worklog are publication scope. Full diff/staged files, hashes, sizes and hooks are checked before normal commit/push. Session snapshots and logs are outside Git at `'+str(Path(plan['models'][0]['snapshot']).parent.parent)+'`.\n'
    log.write_text(text)
    return [ROOT/'README.md',MODELS/'README.md',log,*packages]


def publish(session, paths, do_push):
    repository()
    for path in paths:git('add','--',str(path))
    # *.onnx is generally ignored; explicitly stage only these verified actors.
    for n in ITERATIONS:git('add','-f','--',str(MODELS/f'model_{n}/policy/exported/actor.onnx'))
    git('diff','--cached','--check')
    staged=git('diff','--cached','--name-only').stdout.splitlines()
    allowed={str(p.relative_to(ROOT)) for p in paths if p.is_file()}
    for name in staged:
        assert name in allowed or any(name.startswith(str((MODELS/f'model_{n}').relative_to(ROOT))+'/') for n in ITERATIONS), 'Unrelated staged work: '+name
    patch=git('diff','--cached','--binary').stdout;(session/'full-staged.patch').write_text(patch)
    print(git('diff','--cached','--stat').stdout,flush=True)
    for name in staged:
        p=ROOT/name;assert p.stat().st_size<100*1024*1024 and not name.endswith('.pt')
        if p.suffix in ['.md','.json','.py','.txt','.yaml']:
            text=p.read_text();assert '-----BEGIN PRIVATE KEY-----' not in text
    # Preserve every original tracked model/media/result byte.
    protected=['a3_deploy/a3_deploy_example/models/model_19000','a3_deploy/a3_deploy_example/models/model_29999','docs/assets','docs/evaluations']
    assert not git('diff','HEAD','--',*protected).stdout
    if staged:
        # Show the complete diff to the invoking desktop terminal before committing.
        subprocess.run(['git','-C',str(ROOT),'--no-pager','diff','--cached','--no-ext-diff'],check=True)
        git('-c','user.name=1786089174zth-cell','-c','user.email=zhao14814@gtiit.edu.cn','commit','-m','Publish exact GRU exports13000,14000,14600 and20000')
    commit=repository();save(session/'publication-status.json',{'commit':commit,'pushed':False})
    print('Repository:',ROOT,'\nFeature branch:',BRANCH,'\nOrigin fetch:',git('remote','get-url','origin').stdout.strip(),'\nOrigin push:',git('remote','get-url','--push','origin').stdout.strip(),'\nExact destination: refs/heads/'+BRANCH,flush=True)
    print('Queued commits:\n'+git('log','--oneline',SOURCE+'..HEAD').stdout,flush=True)
    print('Publication files:\n'+git('diff','--name-only',SOURCE+'..HEAD').stdout,flush=True)
    if not do_push:return
    if not remote_inspection(session):
        print('Remote inspection blocked. Commits retained.',flush=True)
        print('Desktop push command:',shlex.join(['git','-C',str(ROOT),'push','--no-follow-tags','origin','refs/heads/'+BRANCH+':refs/heads/'+BRANCH]),flush=True)
        raise RuntimeError('Remote inspection blocked; no publication claimed')
    result=git('push','--porcelain','--no-follow-tags','origin','refs/heads/'+BRANCH+':refs/heads/'+BRANCH,check=False)
    (session/'push.txt').write_text(result.stdout+result.stderr);print(result.stdout+result.stderr,flush=True)
    if result.returncode:raise RuntimeError('Normal push failed; commits preserved, no force/hooks bypass')
    remote=git('ls-remote','origin','refs/heads/'+BRANCH).stdout.split()[0]
    assert remote==commit,'Remote branch not verified at local commit'
    save(session/'publication-status.json',{'commit':commit,'pushed':True,'verified_remote_commit':remote,'branch':BRANCH,'origin':ORIGIN})
    print('PUBLISHED AND REMOTE VERIFIED',commit,flush=True)


def allow_only_owned_work(session):
    """Resume our verified new packages; never stash or stage unrelated edits."""
    status = git('status', '--porcelain', '-z').stdout
    if not status:
        return
    prepared = load(session / 'prepared-documents.json') if (session / 'prepared-documents.json').exists() else {}
    checked = set()
    for entry in status.split('\0'):
        if not entry:
            continue
        name = entry[3:]
        if name in prepared:
            assert sha(ROOT / name) == prepared[name], 'Edited publication document: ' + name
            continue
        for n in ITERATIONS:
            prefix = str((MODELS / f'model_{n}').relative_to(ROOT))
            if name.rstrip('/') == prefix or name.startswith(prefix + '/'):
                if n not in checked:
                    spec = next(x for x in load(session / 'plan.json')['models'] if x['iteration'] == n)
                    verify_package(MODELS / f'model_{n}', spec)
                    checked.add(n)
                break
        else:
            raise RuntimeError('Unrelated local work preserved; review before resuming: ' + name)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--push',action='store_true')
    args=parser.parse_args();session=args.session.resolve();plan=load(session/'plan.json')
    code_commit=repository();assert Path(plan['root'])==ROOT and plan['run_id']==RUN
    assert tuple(s['iteration'] for s in plan['models'])==ITERATIONS
    # Do not stash or overwrite unrelated local work. Clean repeat calls are resumable.
    assert git('merge-base','--is-ancestor',plan['base_commit'],'HEAD',check=False).returncode == 0, 'Snapshot base is not an ancestor of this feature branch'
    allow_only_owned_work(session)
    remote_inspection(session)
    for spec in plan['models']:validate_snapshot(spec)
    packages=[]
    for spec in plan['models']:
        bundle=export_one(session,spec)
        packages.append(package_one(session,spec,bundle,code_commit))
    paths=update_docs(plan,packages)
    save(session/'prepared-documents.json',{str(p.relative_to(ROOT)):sha(p) for p in paths if p.is_file()})
    publish(session,paths,args.push)


if __name__=='__main__':
    try:
        main()
    except Exception as error:
        if '--session' in sys.argv:
            folder=Path(sys.argv[sys.argv.index('--session')+1])
            if folder.is_dir():save(folder/'blocked.json',{'status':'BLOCKED_NOT_PUBLISHED','error':str(error),'behavioral_evaluation_performed':False})
        raise
