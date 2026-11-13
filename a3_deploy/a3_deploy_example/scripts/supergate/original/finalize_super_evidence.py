from pathlib import Path
import csv,json,hashlib,shutil,tarfile,subprocess,difflib,html
import numpy as np,pandas as pd
W=Path(__file__).resolve().parent;O=W.parent/'outputs';E=O/'super-gate3-evaluation';R=Path('/home/user/Desktop/workspace/nohope')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def put(p,obj):p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')
slow=[];slow_summary={};slow_samples=pd.DataFrame(json.loads((O/'super-gate3-slow-tail/samples.json').read_text()))
runs=[('13800','slow-internal-native',0),('14200','slow-internal-native',6),('fusion','slow-internal-fusions',0),('fusion_weighted','slow-internal-fusions',6),('14200_weighted','slow-internal-edited',0)]
for label,folder,offset in runs:
 run=json.loads((W/folder/'run.json').read_text());assert run['return_code']==0 and run['dataset_manifest_sha256']==sha(O/'super-gate3-slow-tail/SHA256.json')
 parts=[]
 for i in range(6):
  r=pd.read_csv(W/folder/f'env-{offset+i}-results.csv').rename(columns={'shot':'shot_id','side':'actual_side'});r['shard_id']=i+1;parts.append(r)
 r=pd.concat(parts).merge(slow_samples,on=['shard_id','shot_id'],validate='one_to_one');assert len(r)==54
 r['model']=label;slow.append(r);slow_summary[label]={'n':54,'contacts':int((r.contacts>0).sum()),'legal':int(r.legal.sum()),'valid_plans':int((r.valid_plans>0).sum()),'complete':int(r.complete.sum()),'fall_indicators':int(((r.min_z<.7)|(r.max_tilt>np.pi/4)).sum())}
pd.concat(slow).to_csv(E/'slow-tail-trials.csv',index=False);put(E/'slow-tail-summary.json',slow_summary)
proof=E/'evidence';proof.mkdir(exist_ok=True)
for name in ['internal-batch-parity.json','internal-batch-benchmark.json','geometry-audit.json','new-model-audit.json','contracts-audit.json','frozen-finalists.json','real-observed-origin-qualified.json','real-plan-ledger.json','real-plan-all-rows.json','real-coverage-filtered.json','gate3-coverage-gap.json','final-source-tests.log','console-tests.log','python-source-tests.log','baseline-field-test.log']:
 shutil.copy2(W/name,proof/name)
for name in ['14200','13800','phase1.0_0.0','14200_bank_1_candidate','dual_edit_fh14200_bh13800']:
 shutil.copy2(W/f'bank-test-{name}.json',proof/f'bank-test-{name}.json')
for name in ['14200','13800']:
 shutil.copy2(W/'horizontal-native'/name/'pp_rally_report.json',proof/f'legacy-full-chain-{name}-report.json')
for folder in ['super-internal-native','super-internal-fusions','super-internal-edited','slow-internal-native','slow-internal-fusions','slow-internal-edited']:
 shutil.copy2(W/folder/'run.json',proof/(folder+'-run.json'))
repro=E/'reproduction';repro.mkdir(exist_ok=True)
shutil.copytree(W/'internal_batch',repro/'internal_batch',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
for name in ['analyze_super_batch.py','build_slow_tail.py','build_candidates.py','bank_eval.py','flight_kernel.py','build_super_factorial.py','fill_factorial.py','freeze_super_dataset.py','build_super_gate3.py']:
 shutil.copy2(W/name,repro/name)
inputs=repro/'constructor-inputs';inputs.mkdir(exist_ok=True)
for name in ['expanded-horizontal-ledger.json','super-gate3-ledger.json','super-factorial-selected.json','super-factorial-requested.json','geometry-audit.json']:
 shutil.copy2(W/name,inputs/name)
shutil.copy2(W/'branch-gate3/fixed-home.yaml',inputs/'fixed-home.yaml');shutil.copy2(W/'serve-14200-0807/ready_reference.npz',repro/'ready_reference.npz')
critical=[W/'internal_batch/native_batch.so',W/'internal_batch/native_batch.cpp',W/'internal_batch/physics.hpp',W/'internal_batch/include/a3_pingpong/pp_planner_input.hpp',W/'model-0807/a3_pingpong.xml']
cmd=json.loads((W/'internal_batch/build-command.json').read_text())
critical += [Path(x) for x in cmd if x.endswith(('.cpp','.so','.so.3.17.0'))]
critical += list((R/'hope_ws/src/hope_planner_cpp/config').glob('model21800_*.yaml'))
critical += [R/'hope_ws/src/hope_planner_cpp/config'/x for x in ['return_physics.yaml','ball_physics_ppq004_20261020.yaml']]
critical += list((R/'agi/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include').rglob('*.hpp'))
critical += list((W/'planner-source/include').rglob('*.hpp'))
critical += list((R/'agi/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/module/mujoco_sim_module/common').glob('gate3_ball*.h'))
pins={str(p):sha(p) for p in dict.fromkeys(critical) if p.is_file()};put(proof/'source-pins.json',pins)
assert sha(W/'internal_batch/native_batch.so')=='3d0e30111ca0285fc62834b151f58e0f0a93d3a29bb51085be93766089af2444'
with tarfile.open(O/'super-gate3-raw-evidence.tar.gz','w:gz',compresslevel=3) as tar:
 for folder in ['super-internal-native','super-internal-fusions','super-internal-edited','slow-internal-native','slow-internal-fusions','slow-internal-edited','internal-batch-parity-1','internal-batch-parity-8']:
  tar.add(W/folder,arcname=folder);tar.add(W/(folder+'.log'),arcname=folder+'.log')
with tarfile.open(O/'super-gate3-source-and-geometry.tar.gz','w:gz',compresslevel=3) as tar:
 tar.add(W/'model-0807',arcname='model-0807')
 tar.add(repro,arcname='reproduction')
 for p in dict.fromkeys(critical):
  if not p.is_file() or str(p).endswith(('.so','.so.3.17.0')):continue
  if p.is_relative_to(R):arc=Path('repo-source')/p.relative_to(R)
  elif p.is_relative_to(W/'planner-source'):arc=Path('planner-source')/p.relative_to(W/'planner-source')
  else:continue
  tar.add(p,arcname=str(arc))
 # Keep support scripts with the exact model compiler/contact profile used.
 for f in ['validate_spin001_serve_mujoco.py','runner_sim_profile.py']:
  tar.add(R/'agi/a3_deploy_example/scripts'/f,arcname='support-scripts/'+f)
baseline=W/'before-scoped';patch=[];changes=[]
paths=[p.relative_to(baseline) for p in baseline.rglob('*') if p.is_file()]
paths += [Path('agi/a3_deploy_example/scripts/tests/test_prepare_serve_policy_bundle.py')]
paths += [Path('agi/a3_deploy_example/assets/a3_runtime/serve/motions')/(n+ext) for n in ['v23-14200','v23-14200-fusion','v23-14200-best'] for ext in ['.csv','.recipe.json']]
for rel in dict.fromkeys(paths):
 before=baseline/rel;after=R/rel
 if before.exists():old=before.read_text()
 elif str(rel).endswith('test_prepare_serve_policy_bundle.py'):
  p=subprocess.run(['git','-C',str(R),'show','HEAD:'+str(rel)],capture_output=True,text=True);old=p.stdout if p.returncode==0 else ''
 else:old=''
 new=after.read_text() if after.exists() else ''
 if old==new:continue
 patch.extend(difflib.unified_diff(old.splitlines(keepends=True),new.splitlines(keepends=True),fromfile='a/'+str(rel) if old else '/dev/null',tofile='b/'+str(rel) if new else '/dev/null'))
 changes.append({'path':str(rel),'snapshot_before_sha256':hashlib.sha256(old.encode()).hexdigest() if old else None,'after_sha256':sha(after) if after.exists() else None})
(O/'model14200-source-changes.patch').write_text(''.join(patch));put(O/'source-change-manifest.json',changes)
(repro/'README.txt').write_text('''复现说明
正式评测入口是 internal_batch/run.py：一个进程，MuJoCo mjb_apply 内部线程池。
固定 12 个环境，前 6 个环境使用 policy0，后 6 个使用 policy1；--single-model 为 6 个环境。
环境各自顺序运行一个 shard，保持连续状态。物理 1 kHz，策略 50 Hz，球观测 250 Hz。
run.py 200 8 OUTPUT 跑主集；run.py 9 8 OUTPUT --dataset SLOW_DATASET 跑慢球集。
--policy0/--policy1/--label0/--label1 可指定策略目录和记录名。
Python 3.12 / numpy / PyYAML / MuJoCo 3.17.0；C++20 / Eigen / yaml-cpp / ONNX Runtime 1.19.2。
编译参数见 internal_batch/build-command.json，源码字节见 evidence/source-pins.json。
原机用 distrobox enter hope -- /home/user/Documents/Codex/2026-11-04/new-chat/work/venv/bin/python。
脚本保留原机绝对路径，跨机器需把路径指向源码快照、model-0807、ready_reference.npz、数据集和策略目录。
source-and-geometry 压缩包包含生产头文件、参与编译的源文件及几何；外部编译器、Eigen/ORT/MuJoCo 等依赖不随包分发。
数据集可直接复用，无需重新随机生成。SHA256.json 冻结 YAML、ledger 和所有构造尝试。
构造历史脚本及输入另存，使用时按脚本的 work 布局放置；这里不声称是跨机零配置安装器。
super-gate3-v1/reproduction 中的 run_isolated 等文件是早期外部多进程实验快照，不是本次计分后端。
v1 design.json 的旧 scope 字符串包含 Full native Runner；应以本次 run.json 为准：仅生产策略/Planner核心/物理，不含外层 Runner/DDS/HAL。
''')
summary=json.loads((E/'summary.json').read_text());groups=json.loads((E/'groups.json').read_text())
selection={'selected':'14200_bank_1_candidate','policy_sha256':'95e63b544763ee52e9d6dfa45e0584e4205479a09eba2ba31cf2572e9e9be7e8','basis':'Highest legal count among five pre-frozen candidates on this benchmark, no fall indicators. Not a globally optimal or hardware-qualified model.','main_legal':800,'main_n':1200,'slow_legal':42,'slow_n':54,'paired_vs_original':{'wins':17,'losses':10},'tradeoff':'Legal landing error p90 worsened from 0.4000 m to 0.4392 m; p95 also worsened. Only +7/1200 legal returns.','deployment':'Fusion and best staged only. Direct 14200 deployment owned by separate chat; this run did not change hardware state.'};put(O/'best-selection.json',selection)
names={'13800':'13800 原版','14200':'14200 原版','fusion':'14200+13800 融合','fusion_weighted':'双模型权重调整融合','14200_weighted':'14200 权重调整（本次最佳候选）'}
table=''
for model,r in summary.items():
 table+=f'<tr><td>{names[model]}</td><td>{r["contacts"]}/1200</td><td>{r["legal"]}/1200 ({r["legal_pct"]:.2f}%)</td><td>{r["core"]["legal"]}/972</td><td>{slow_summary[model]["legal"]}/54</td><td>{r["landing_error_legal_median_m"]*100:.1f} / {r["landing_error_legal_p90_m"]*100:.1f} / {r["landing_error_legal_p95_m"]*100:.1f} cm</td></tr>'
side=''
for model in ['13800','14200','14200_weighted']:
 rs={r['group']:r for r in groups if r['model']==model and r['axis']=='planned_side_name'}
 side+=f'<tr><td>{names[model]}</td><td>{rs["FH"]["legal"]}/{rs["FH"]["n"]}</td><td>{rs["BH"]["legal"]}/{rs["BH"]["n"]}</td></tr>'
doc=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Super Gate3 与 14200 适配评测</title><style>body{{max-width:1100px;margin:40px auto;padding:0 22px;font:16px/1.7 system-ui,sans-serif;color:#17212b}}h1,h2{{line-height:1.3}}table{{border-collapse:collapse;width:100%;font-size:14px}}td,th{{padding:9px;border-bottom:1px solid #ccd5dd;text-align:left}}th{{background:#eef3f7}}img{{max-width:100%;height:auto}}code{{overflow-wrap:anywhere}}.note{{padding:16px;background:#fff2ce;border-left:4px solid #bd8e11}}a{{color:#075c9b}}</style><body>
<h1>Super Gate3 与 14200 适配评测</h1><p>2026-11-08 · 离线物理仿真 · 5 个预先冻结版本 × 1,254 条球路，共 6,270 次测试</p>
<p><b>MuJoCo 内部并行已实现。</b>当前合法回球数最多的是 14200 权重调整候选；融合版没有超过原版。旧测试确实过窄，新集暴露了高球与快球的明显短板。</p>
<p class="note">本报告的成绩是物理接触/回球指标，不是正式 Gate3 全链路 PASS，也不是实机资格。best 表示本次有限候选中的最高合法回球数；相对原版只增加 7/1200，落点误差尾部反而更大。</p>
<h2>结果</h2><table><tr><th>模型</th><th>碰到球</th><th>主集合法回球</th><th>81 格主样本合法</th><th>慢球补充合法</th><th>合法回球落点误差 中位/P90/P95</th></tr>{table}</table>
<p>合法回球：球拍接触后首次落台在对方半台且之前未碰网。多次接触按一条球路计；未规划、没碰到、出界和未结束轨迹均保留在分母。落点误差只统计合法回球，对比时有成功样本选择偏差。目标回球落点固定为 [2.055,-0.7625] m。</p>
<table><tr><th>模型</th><th>正手合法回球</th><th>反手合法回球</th></tr>{side}</table><p>按实际 Planner 选边统计；两个模型有 1 条选边不同。14200 正手更强，13800 反手更强，但融合切换后的状态及动作轨迹不是两个单模型分数的简单拼接。融合失败原因尚未被单独因果定位。</p>
<p>原版 14200 高球合法 178/410，13800 为 159/410；快球分别 190/402、180/402。两个原版在高快组合仍有大量失败，详见每格 12 条的结果。best 相对原版逐球对比：新增成功 17 条，退化 10 条；不能推断统计显著或全局最优。</p>
<img src="super-gate3-evaluation/baseline-81-cells.png" alt="81 个组合每格 12 条的合法回球结果">
<h2>球路覆盖</h2><p>主集 972 条 = 目标横向 3 × 高度 3 × 来球速度 3 × 水平角度 3 × 每格 12 条；另有 228 条出发位置/边界球，共 1,200 条不同初始状态。六个 shard 各 200 条，每个 shard 每格 2 条，连续发球。样本按物理可行性预先选择，未使用策略成绩；全部构造尝试和排除原因保留。不是现场自然分布的独立同分布抽样。</p>
<table><tr><th>变量</th><th>定义/范围</th></tr><tr><td>目标横向</td><td>相对 HOME 的 y：[-0.68,-0.32]、[-0.32,-0.15]、[-0.15,0.28] m；图的左/中/右按坐标区间命名</td></tr><tr><td>目标高度</td><td>世界 z：[0.8,1.0]、[1.0,1.2]、[1.2,1.45] m</td></tr><tr><td>来球速度</td><td>接触平面处三维速度模长：&lt;2.5、2.5–4、&gt;4 m/s；主集实际 1.801–5.098 m/s</td></tr><tr><td>来球角度</td><td>atan2(vy,-vx)：&lt;-5°、[-5°,5°]、&gt;5°；含边界集实际 -41.74° 至 +29.72°</td></tr><tr><td>出发点</td><td>x 2.000–2.798 m；y -1.943–0.305 m；不是统一从中间出发</td></tr><tr><td>上升/下降</td><td>全部 1200 条含 797 上升、403 下降；初始旋转也变化</td></tr></table>
<p>高快核心组合有 108 条，其水平速度 3.827–4.889 m/s；不是靠较大的竖直分量把慢横向球标成快球。原生 Planner 在 971/972 条核心样本上预测到同一分格；SG3v1-01-158 的预测高度从构造值 1.176 m 变为 1.246 m，保留在原始分格，没有事后换题或删掉。</p>
<img src="super-gate3-evaluation/coverage.png" alt="出发点、速度高度及角度横向覆盖">
<h2>历史真球与仍未覆盖的范围</h2><p>读取 10 个历史 session 的 278 条有效 Planner 预测，其中 268 条在当前位置支持范围内；10 条越界预测原样保留。录制数据中 182 条来球经过 x=2.4 m 平面的观测通过局部拟合及通向球网的连续性检查，y 范围 -1.440 至 0.322 m、世界高度 0.869 至 2.028 m、速度 1.565 至 6.392 m/s、角度 -28.36° 至 17.47°。这是实测经过平面的状态，不是恢复出来的对手出拍点；不能用它声称真实出发点已全部重建。PoseArray 包含机器人点，本次仅在来球段按空间范围排除本方机器人点。</p>
<p>主集慢球下限漏掉 49 条历史支持范围内的预测。新增独立慢球集：48/49 个目标在位置 2.5 cm、速度 0.06 m/s、角度 1.5° 的构造容差内得到可行近似，加 6 条扰动球，共 54 条；实际 1.345–1.808 m/s。旋转是构造变量，不是实测旋转，故这是近似合成球路而非逐条实战回放。</p>
<p>剩余缺口：1 条 1.233 m/s 的历史目标未被优化器构造成功（不代表物理不可能）；主集实际高度边缘还漏掉 3 条较低和 1 条较高的支持内预测；10 条支持外预测不能假装已经被正常能力范围覆盖。81 个粗分格齐全不等于连续空间、旋转、噪声、出球目标和时序所有角落已穷尽。</p>
<h2>训练分支和 URDF</h2><p>核对 origin/main_urdf_2 @ <code>4e406be64451301b7d8abba6f5a2c260272f6f87</code>：训练 bank 确实增加了 351,000 行，正手 216,000、反手前方 108,000、其他反手 27,000，补齐速度×高度的九格；训练集合计 853,053 行。新增行属于合成覆盖扩展，不等于同数量独立实战球。14200 声明 stationary_reach_coverage_stable_v3，反手面接触点局部 y=-0.013208 m；ABI 110/31、控制与反馈契约同 13800。</p>
<p>gate3_stationary_aim 分支主要改出球目标/顺序，旧 26/30 球接触速度约 2.46–2.48 m/s、高度约 1.04 m，无法据此证明宽范围能力。旧出发点 y 有变化，但 x、z、vx、vy、vz 几乎固定，不能说严格全部同一个点，也确实缺少独立变化的入射角和速度高度组合。</p>
<p>测试使用 A3P_P1_0807_OP3_PR120 几何：31 关节顺序一致，质量由 58.2879 kg 变为 57.6306 kg，静态球拍站点差约 [0.154,0.059,0.475] mm；FK、刚体质量、初始自碰撞和拍面接触检查记录在 geometry-audit.json。仅测试模型更新，未替换机器人现场 URDF。</p>
<h2>内部并行与测试边界</h2><p>一个 Python 宿主进程调用 C++ <code>mjb_makeBatch / mjb_apply</code>，12 个独立 mjData 环境，8 个 MuJoCo 内部工作线程。每个环境有自己的策略、Planner、观测历史和 thread_local 仿真时钟。生产 PpPolicy、IncomingTrajectory、物理估计、未来接触预测、选边及回球求解核心与生产球碰撞方法参与计算；1 ms 物理、20 ms 策略、4 ms 球观测。这里没有启动 12 套 Runner/ROS 程序。参见 <a href="https://mujoco.readthedocs.io/en/stable/python.html">MuJoCo 官方 Batch 文档</a>。</p>
<p>同一原生核心的 24 条小样本串行/8线程对照，24 个结果和轨迹 CSV 逐字节一致；87.311 s → 21.754 s，4.0135 倍。两个原版共 2,400 条完整测试用时 250.085 s。此次 MuJoCo 3.17.0；旧全链路安装使用 3.1.6，成绩不能当成同版本后端的严格等价证明。</p>
<p>观测使用模拟真值，固定 8 ms 指令交付模型，初始状态来自共同 WAIT 参考并各自热身 20 s。未覆盖真实 mocap 噪声/丢包/抖动、DDS、HAL、外层 Runner 管理生命周期和墙钟性能。五个模型均无本报告定义的跌倒指标（骨盆 &lt;0.7 m 或倾角 &gt;45°），不等于通过原正式稳定性门槛。主集每个模型 1200/1200 都产生有效规划，但大量没接到球。</p>
<p>13800 的 SG3v1-05-192 在收尾时仍未自然结束，零接触、碰网后持续台面接触，按失败保留；双模型权重融合的 SG3v1-04-199 也未自然结束（零接触，按失败保留）；其余主集轨迹完成。执行器仍按原限幅夹紧，13800/14200/best 分别有 952/813/816 条球路出现请求扭矩超过限幅，最高请求约 5.34/5.17/5.15 倍（以原始 CSV 精确值为准），不表示实际施加该扭矩。日志未出现数值不稳定警告；本版没有单独导出 MuJoCo warning 计数器。旧原生全链路测试的正式失败报告一并保留，未放宽验收标准。</p>
<h2>模型产物与部署状态</h2><p>已输出直接适配包、普通融合包、权重调整 best 候选包，均为 aarch64 部署包，各自绑定专属 serve CSV、recipe 和哈希。三套发球验证均为五种情形 PASS，正常/内核各重复三循环，交接 q/dq/tau/kp/kd 连续。重新打包时补齐 recipe 源 CSV，排除 .DS_Store 并规范静态文件读取权限；包内 CSV 已由 recipe 逐字节复现。相关源代码检查 58/58 通过。Console 构建/类型检查通过；完整 UI 测试 115/116、field operations 59/60，两个既有失败分别是中文文案与 stop 调用次数，未在本任务中修补无关行为。</p>
<p>best 采用预先开发的 6 维、随球钟变化的有界第一层权重编辑，WAIT 点保持等价；不改 PD、反馈顺序、球物理或正式阈值。直接按比例混合两网权重导致较大跟踪误差，已淘汰；普通融合使用正手 14200、反手 13800 的平滑动作输出门控。五个最终候选在大样本前冻结，没有依据本次失败球继续改权重。</p>
<p>旧 USB fusion 已先备份再移除，新模型包另行存放。另一个部署聊天完成 14200 原版安装/选择并报告 STOPPED/IDLE、无 Runner/Planner；这里只负责离线数据与 staged 候选，未启动或切换现场模型。当前现场基线应继续按原版 14200 识别。</p>
<h2>可核对文件</h2><ul><li><a href="super-gate3-evaluation/all-trials.csv">主集逐球结果（6000 行）</a>、<a href="super-gate3-evaluation/81-cells.csv">81 格明细</a>、<a href="super-gate3-evaluation/slow-tail-trials.csv">慢球逐球结果（270 行）</a></li><li><a href="super-gate3-v1/design.json">冻结主集设计</a>、<a href="super-gate3-slow-tail/design.json">慢球补充集</a>、<a href="super-gate3-evaluation/evidence/real-observed-origin-qualified.json">实测来球与缺失记录</a></li><li><a href="super-gate3-raw-evidence.tar.gz">原始轨迹/结果/日志</a>、<a href="super-gate3-source-and-geometry.tar.gz">源代码与测试几何快照</a></li><li><a href="bundle-verification.json">部署包验证</a>、<a href="best-selection.json">best 选择及取舍</a>、<a href="model14200-source-changes.patch">本次源码差异</a></li></ul>
<p>主集 SHA256 manifest：<code>1b2d08d4d0eb16e9244125e0019378c82c196ec2e2f96abdabcac49072485c59</code><br>内部 batch 库：<code>3d0e30111ca0285fc62834b151f58e0f0a93d3a29bb51085be93766089af2444</code></p></body></html>'''
# Use exact measured requested-torque ratio in the prose.
doc=doc.replace('5.34/5.17/5.15', '/'.join(f'{summary[m]["max_requested_torque_ratio"]:.2f}' for m in ['13800','14200','14200_weighted']))
(O/'super-gate3-report.html').write_text(doc)
for directory,title in [('model14200-direct-portable','14200 原版直接适配'),('model14200-fusion','14200+13800 普通融合'),('model14200-best','14200 权重调整候选')]:
 p=O/directory;rec=json.loads((p/'receipt.json').read_text());(p/'README.txt').write_text(title+'\n\n身份以 receipt.json / mdu_entry.json / hdu_entry.json 为准。\n本目录是部署包，不是自动安装器。融合与 best 仅 staged，未部署到机器人。\n完整评测见同级 super-gate3-report.html。发球仿真 PASS 不等于实机打球资格。\n'+json.dumps(rec['identity'],ensure_ascii=False,indent=2)+'\n')
print('Final evidence/report written',flush=True)
