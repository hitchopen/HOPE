from pathlib import Path
import subprocess,shutil,json
W=Path(__file__).resolve().parent;R=Path('/home/user/Desktop/workspace/nohope');B=W/'branch-gate3';S=W/'planner-source'
if not S.exists():shutil.copytree(R/'hope_ws/src/hope_planner_cpp',S)
patch=subprocess.check_output(['git','-C',str(R),'show','--format=','cedab0d07','--','hope_ws/src/hope_planner_cpp'])
(W/'planner-sequence.patch').write_bytes(patch)
subprocess.run(['git','apply','-p4','--directory=planner-source',str(W/'planner-sequence.patch')],check=True,cwd=W)
for n,p in [('fixed-home.yaml','a3_deploy/a3_deploy_example/config/gate3/scenarios/fixed-home.yaml'),('random30_seed20261031.csv','a3_deploy/a3_deploy_example/config/gate3/land_sequences/random30_seed20261031.csv')]:
 (B/n).write_bytes(subprocess.check_output(['git','-C',str(R),'show','origin/gate3_stationary_aim:'+p]))
# Keep the local timing/alignment/cleanup harness, add only branch aim options.
source=(B/'run_gate3.py').read_text();s=(W/'run_gate3.py').read_text().replace('import argparse\n','import argparse\nimport csv\n')
s=s.replace('\ndef digest(path):','\n'+source[source.index('# Planner landing aim'):source.index('\ndef digest(path):')]+'\ndef digest(path):')
s=s.replace('\ndef parser():','\n'+source[source.index('def aim_point('):source.index('\ndef parser():')]+'\ndef parser():')
args=source[source.index('    p.add_argument("--land-x"'):source.index('    return p',source.index('    p.add_argument("--land-x"'))]
s=s.replace('    return p\n',args+'    return p\n',1)
start=source.index('    aim, aim_env = aim_point(');end=source.index('    inspection =',start)
print(source[start:end])
# Branch builds the sequence after out creation and before native inspection.
s=s.replace('    inspection = native_inspect(dist, policy)',source[start:end]+'    inspection = native_inspect(dist, policy)')
s=s.replace('        "PP_DIST": str(dist),','        **aim_env,\n        "PP_DIST": str(dist),')
s=s.replace('"resolved_scene_environment": scene_env,','"resolved_scene_environment": scene_env, "aim_point": aim, "land_sequence": sequence,\n               "planner_sha256": digest(Path("'+str(W/'planner-build/hope_planner_cpp_node')+'")),')
s=s.replace("'/home/user/Documents/Codex/2026-11-04/new-chat-6/work/hitter.sh'",repr(str(W/'hitter-suite.sh')))
(W/'run_gate3_suite.py').write_text(s)
old=Path('/home/user/Documents/Codex/2026-11-04/new-chat-6/work')
s=(old/'hitter.sh').read_text().replace(str(old/'rally.sh'),str(W/'rally-suite.sh'));(W/'hitter-suite.sh').write_text(s)
s=(old/'rally.sh').read_text();s=s.replace('PLANNER_CPP_EXE="$PLANNER_CPP_PREFIX/lib/hope_planner_cpp/hope_planner_cpp_node"','PLANNER_CPP_EXE="'+str(W/'planner-build/hope_planner_cpp_node')+'"')
branch=(B/'pp_gate3_rally.sh').read_text();start=branch.index('# Entry-level aim override');end=branch.index('# Viewer-only',start)
s=s.replace('echo "[g3r] production hope_planner_cpp',branch[start:end]+'echo "[g3r] production hope_planner_cpp')
s=s.replace('$QUESTION_FIXTURE_ARGS \\', '$QUESTION_FIXTURE_ARGS $LAND_SEQUENCE_ARGS \\');(W/'rally-suite.sh').write_text(s)
