from pathlib import Path
import json,html
W=Path(__file__).resolve().parent;O=W.parent/'outputs/super-gate3-detailed-20261108'
def read(n):return json.loads((O/n).read_text())
out=read('outcomes.json');errors=read('racket-error-statistics.json');post=read('posture-statistics.json');lands=read('landing-statistics.json');paired=read('paired-outcomes.json');cond=read('condition-statistics.json')
models=[x['model'] for x in out if x['dataset']=='main']
names={'13800':'13800 原版','14200':'14200 原版','fusion':'侧别融合','14200_weighted':'14200 权重版（上一版 best）','fusion_weighted':'双模型权重融合','pvn_joint_14200':'新：14200 联合 P/V/N 权重','pvn_side_fusion':'新：分侧 P/V/N 权重融合','pvn_strike_residual':'新：击球窗口残差融合'}
def pick(rows,**kw):return next(r for r in rows if all(r[k]==v for k,v in kw.items()))
def e(m,met,event='planned',**kw):return pick(errors,dataset='main',model=m,event=event,axis=kw.get('axis','ALL'),group=kw.get('group','ALL'),metric=met)
def p(m,met):return pick(post,dataset='main',model=m,axis='ALL',metric=met)
def f(v,n=2):return '—' if v is None else f'{v:.{n}f}'
def ms(r,n=2):return f"{f(r['mean'],n)} ± {f(r['std'],n)}"
def table(head,rows):return '<div class="scroll"><table><thead><tr>'+''.join('<th>'+html.escape(str(c))+'</th>' for c in head)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(c))+'</td>' for c in row)+'</tr>' for row in rows)+'</tbody></table></div>'
parts=['''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Super Gate3 · 线程、拍面误差、落点与姿态</title><style>body{font:16px/1.65 system-ui,sans-serif;background:#f3f5f7;color:#1b2935;margin:0}main{max-width:1400px;margin:auto;padding:32px}h1{font-size:32px;line-height:1.3}h2{margin-top:42px}a{color:#0966ae}section,.note{background:white;padding:22px;border-radius:12px;margin:20px 0}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:10px 12px;border-bottom:1px solid #dfe5ea;text-align:right;white-space:nowrap}th:first-child,td:first-child{text-align:left}th{background:#e8eef4}img{width:100%;height:auto}.scroll{overflow-x:auto}.note{border-left:4px solid #168451}small{color:#52616d}code{background:#edf1f5;padding:2px 5px}li{margin:6px 0}pre{white-space:pre-wrap}footer{color:#52616d}</style><main><h1>Super Gate3：把合法率之外的差别量出来</h1><p>2026-11-08 · 离线 MuJoCo 3.17.0 内部线程池 · 0807 机器人几何 · 每模型 1,200 主集 + 54 慢球尾部样本</p>''']
selection=read('selection-review.json') if (O/'selection-review.json').exists() else {'conclusion':'结果整理中；请以最终 selection-review.json 为准。','findings':[]}
parts.append('<div class="note"><b>'+html.escape(selection['conclusion'])+'</b><ul>'+''.join('<li>'+html.escape(x)+'</li>' for x in selection['findings'])+'</ul></div>')
parts.append('<section><h2>1. 8 线程不是上限</h2><p>同一批 24 个环境，每环境 8 次投球，另含 20 秒 WAIT 热身；每档跑两次，顺序 8/24/12/16/16/12/24。所有球轨迹、得分和新增诊断 CSV 在不同线程数下逐字节一致。重复球路仅用于测吞吐，不计入覆盖样本。</p>')
bench=json.loads((W/'thread-scaling.json').read_text())
parts.append(table(['线程','两次耗时（秒）','中位数（秒）','相对 8 线程速度'],[[t,' / '.join(f(r['seconds']) for r in bench['runs'] if r['threads']==t),f(bench['median_seconds'][str(t)]),f(bench['median_seconds']['8']/bench['median_seconds'][str(t)])+'×'] for t in [8,12,16,24]]))
parts.append('<p>本机为 24 个物理核心的混合架构 CPU，24 是本次测试最快档，不是 MuJoCo 的固定上限。单进程、原生 batch 线程池；有效并行度还受环境数限制。五模型批次为 30 环境，三新候选批次为 18 环境，后者申请 24 线程时实际最多并行 18 个环境。没有同时启动多套仿真进程，也未用 GPU。</p><p><a href="https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjb-makebatch">MuJoCo 官方 batch API</a> · <a href="evidence/thread-scaling.json">原始计时</a></p></section>')
parts.append('<section><h2>2. 同球路主集结果与三项拍面误差</h2><p>每模型主集 1,200 个样本，含 81 个组合 × 12 条不同轨迹，以及 228 条出发点/边界样本；54 条慢球尾部另列，未混成一个加权总分。下表误差在原始 Planner 计划击球时刻采样（下一 1 ms tick，误差不超过 1 ms），包含全部漏球。均值 ± 样本标准差（ddof=1），不是标准误差。</p>')
rows=[]
for m in models:
 r=pick(out,dataset='main',model=m);slow=pick(out,dataset='slow',model=m);pair=pick(paired,dataset='main',model=m)
 rows.append([names[m],f"{r['contacts']}/1200",f"{r['legal']}/1200",f"{slow['legal']}/54",ms(e(m,'p_error_cm')),ms(e(m,'v_error_mps'),3),ms(e(m,'n_error_deg')),f"+{pair['wins_vs_14200']} / −{pair['losses_vs_14200']}"])
parts.append(table(['模型','触拍','合法','慢球合法','位置 cm','速度 m/s','拍面法向 °','对原14200 新赢/新输'],rows))
parts.append('<img src="racket-errors.png" alt="计划时刻三项误差箱线图"><p>箱线图为便于比较隐藏离群点，完整最大值、P90/P95、XYZ 有符号偏差及各组样本数在 CSV 中。位置为统一物理拍面中心：正手使用 site，反手使用 local Y −0.013208 m；13800 原训练 site 定义不同，因此同时导出 legacy site 误差供核对。速度为实际运动学速度。法向为双面球拍平面夹角（先将实测法向翻向目标半球），不把两面法向符号差当作 180° 误差。</p>')
parts.append('<h3>实际第一次触拍时刻：仅触拍样本</h3><p>接触点位置/速度取 MuJoCo 接触几何，与原始 Planner 目标比较；法向取实际拍面。不能把此表与上面的全样本误差直接混算。生产接触步时间分辨率为 1 ms，接触事件 FK/qvel 存在步内时间差，速度用于离线近似诊断，不作为亚毫秒精确实测。</p>')
parts.append(table(['模型','触拍样本数','实际接触点位置 cm','实际接触点速度 m/s','拍面法向 °','接触时差 mean ± std ms'],[[names[m],e(m,'surface_position_error_cm','contact')['n'],ms(e(m,'surface_position_error_cm','contact')),ms(e(m,'surface_velocity_error_mps','contact'),3),ms(e(m,'n_error_deg','contact')),ms(e(m,'dt_ms','contact'))] for m in models]))
parts.append('<p>少量第一次物理触拍发生在计划时刻很久之后：14200 的最大迟到为 3.23 秒，拉大时差 std。它们全部是非法回球，没有抬高合法数；仍完整保留在上表，不能把它们视为正常的计划挥拍接触。另用 |时差|≤100 ms 作诊断分组（不改变评分标准）：</p>')
timing=read('contact-timing-statistics.json');rows=[]
for m in models:
 r=pick(timing,dataset='main',model=m,timing_group='abs_dt_le_100ms');late=pick(timing,dataset='main',model=m,timing_group='abs_dt_gt_100ms')
 rows.append([names[m],r['n'],late['n'],ms(r['metrics']['surface_position_error_cm']),ms(r['metrics']['surface_velocity_error_mps'],3),ms(r['metrics']['n_error_deg']),ms(r['metrics']['dt_ms'])])
parts.append(table(['模型','时差≤100ms 样本','时差>100ms 样本','接触点位置 cm','接触点速度 m/s','法向 °','时差 mean ± std ms'],rows))
parts.append('<p><a href="contact-timing-statistics.json">按触拍时差分组</a> · <a href="late-contacts.csv">全部迟到触拍明细</a></p>')
parts.append('<p><a href="racket-error-statistics.csv">所有均值、std、P90/P95 和 XYZ 分组统计</a> · <a href="racket-errors.csv">逐次计划/接触误差</a> · <a href="81-cells.csv">81 格结果</a></p></section>')
parts.append('<section><h2>3. 落点分布与尾部</h2><p>固定目标点 (2.055, −0.7625) m。优先记录触拍后的首次实体球台接触；无实体落台的回球，用实测轨迹首次下降穿过球心高度 z=0.7801 m 的位置作为投影。投影只用于看出界方向，不算合法落台。漏球单独计数，不补零进落点均值。</p><img src="landing-distributions.png" alt="落点分布">')
rows=[]
for m in models:
 r=pick(lands,dataset='main',model=m)
 rows.append([names[m],r['physical_table_contacts'],r['projected_off_table'],ms(r['dx_mean_std'],3),ms(r['dy_mean_std'],3),f(r['radial_all_measurable']['p95'],3),f(r['radial_legal_only']['median'],3),f(r['radial_legal_only']['p95'],3)])
parts.append(table(['模型','实体落台','投影','X偏差 mean ± std m','Y偏差 mean ± std m','全部可测 P95 m','合法落点中位 m','合法落点 P95 m'],rows))
parts.append('<p>“合法落点”是条件统计，不能掩盖漏球和出界；因此同时给全部可测回球的分布、样本数和失败计数。图外点保留在原始数据中。<a href="landing-points.csv">逐球落点与来源</a> · <a href="landing-statistics.json">分布、协方差、过短/过长/侧出界计数</a></p></section>')
parts.append('<section><h2>4. 倒地、侧身、收拍残余转角</h2><p>转角相对每个环境 20 秒热身后的初始 WAIT 朝向，躯干和骨盆分别记录。峰值取每 4 ms 记录，统一收拍点取计划击球后 +0.6 秒（50 Hz 记录中最近点，最大差 12 ms）；最后一球额外 drain 不影响统一收拍点。侧身本身不等于倒地，也没有未经验证地把某个转角设成失败线。</p>')
rows=[]
for m in models:
 r=pick(out,dataset='main',model=m)
 rows.append([names[m],f"{r['falls']}/1200",f(r['min_pelvis_height_m'],3),f(r['max_tilt_deg']),ms(p(m,'peak_torso_yaw_deg')),ms(p(m,'peak_pelvis_yaw_deg')),ms(p(m,'recovery_torso_yaw_deg')),r['recovery_abs_torso_above15_trials']])
parts.append(table(['模型','倒地指标','最低骨盆高度 m','最大倾斜 °','峰值躯干 |yaw| °','峰值骨盆 |yaw| °','+0.6s 躯干 yaw °','收拍 |yaw| >15° 次数'],rows))
parts.append('<img src="body-orientation.png" alt="侧身和收拍转角分布"><p>倒地指标沿用骨盆高度 &lt;0.7 m 或倾斜 &gt;45°；它是离线姿态指标，未增加身体撞地判据，也不是实机站稳认证。还导出躯干-骨盆扭转、roll/pitch、相对名义 HOME 的 XY 漂移及力矩饱和时间占比。原始请求力矩越限后仍按相同执行器限值裁剪；没有改 PD 或放大实际执行器能力。</p><p><a href="posture-statistics.csv">完整姿态统计</a> · <a href="trials.csv">逐球姿态峰值、恢复和得分</a></p></section>')
parts.append('<section><h3>两项新候选的倒地起点</h3><p>联合权重版在 shard 4 第 194 球首次触发，直到第 200 球的 7 个连续窗口受影响；击球窗口融合在 shard 1 第 177 球首次触发，此后 24 个窗口受影响。不是 7 次和 24 次独立摔倒。两者各有一个首次起点，后续低姿态持续；全部后续球仍计入统计，未删掉这些失败。</p><img src="rejected-candidate-falls.png" alt="两项新候选的倒地时间序列"><p><a href="fall-onsets.json">起点与样本 ID</a> · <a href="pvn_joint_14200-fall-window.csv">联合权重版时间窗</a> · <a href="pvn_strike_residual-fall-window.csv">窗口融合时间窗</a></p></section>')
parts.append('<section><h2>5. 分正反手和球路查短板</h2>')
rows=[]
for m in models:
 rows.append([names[m]]+[str(pick(cond,dataset='main',model=m,axis='side_name',group=s)['legal'])+'/'+str(pick(cond,dataset='main',model=m,axis='side_name',group=s)['n']) for s in ['FH','BH']])
parts.append(table(['模型','正手合法','反手合法'],rows))
parts.append('<img src="height-speed-coverage.png" alt="各模型高度与速度分组合法率">')
parts.append('<p>同一球在边界处可能因前序动作/机器人状态变化选择不同侧，侧别分母因此可能差 1。固定样本 ID 的成对胜负不受这个问题影响。各速度、高度、角度、左右目标分组见 <a href="condition-statistics.json">条件统计</a>；每格 12 条是有覆盖的初步比较，不能据此证明连续范围没有 corner case。</p></section>')
parts.append('''<section><h2>6. 沿用之前经验，确实修改网络 weight</h2><p>上一轮 14200 权重版已经修改 MLP 第一层的 512 个 bias 和 512 个 time-to-strike 列权重，共 1,024 个参数；不是只换模型混合比例。方法为 d=W₀[:,103:109]·Δ(p,v)，b₀+=d，W₀[:,109]−=d/0.82，使 WAIT 的 tts=0.82 输入等价，外部 Planner 目标保持不变。</p><p>此次三候选在新仿真结果出现前冻结：使用既存训练 bank 的 DEV 有限差分，联合位置/速度/拍面法向残差，法向残差倍率 2.5；纯 14200 联合校准、14200 正手与 13800 反手分别校准、以及击球附近最多加入 25% 的 13800 反手残差。后者在 tts 0.35→0.20 秒渐入，0.20→0 秒全窗，0→−0.10 秒退出，WAIT 仍走 14200。</p><p>这次拟合未使用 Super Gate3 逐球残差，也没有改外部 target、PD、接触物理或合法标准。但之前结果参与了方法选择，因此这套 benchmark 已被用于候选选择，不能称为全新盲测。DEV 线性预测改善不是实测改善。整网平均、减掉朝向反馈或改腰部输出增益在此前经验中没有稳定全指标胜出，因此没有把它们当捷径。</p><p><a href="evidence/pvn-followup-proposals.json">冻结参数与 DEV 来源</a> · <a href="selection-review.json">本轮选择和取舍</a> · <a href="paired-outcomes.json">成对新赢/新输及未校正描述性统计</a></p></section>''')
parts.append('''<section><h2>7. 可复核性与验证范围</h2><ul><li>新增只读 FK/姿态观测后，原有 6,000 主集与 270 慢球测试的球轨迹和得分 CSV 全部逐字节相同。</li><li>检查位置/速度范数恒等式、单位法向和夹角公式、计划采样时间偏差、触拍样本数、合法回球落台来源、81 格每格 12 条以及样本唯一性；见 <a href="analysis-validation.json">分析校验</a>。</li><li>运行 production 原生策略和因果预测/Planner 核心，1 kHz 接触物理、固定 8 ms 投递延迟、各环境独立仿真时钟。没有覆盖 DDS、HAL、进程生命周期、真实调度/动捕抖动或实机动作。</li><li>0807 几何保持与前轮一致；模型契约、物理、得分定义保持不变。旧 full-chain Gate3 的失败仍保留，内部 batch 通过不能把它改记为通过。</li><li>球路覆盖仍有边界：这版主要在声明位置范围内；真实记录中的少数更低、更高及一个约 1.233 m/s 的慢球目标未被成功构造。慢球尾部是合成近似，不是原始实机传感器逐帧重放。</li></ul><p><a href="evidence/detailed-main-run.json">5 模型主集运行收据</a> · <a href="evidence/detailed-candidates-run.json">3 新候选运行收据</a> · <a href="evidence/detailed-full-parity.json">全量物理结果不变校验</a> · <a href="reproduction/README.txt">复现说明</a></p></section><footer>所有结论限定于已测模型、固定数据集与此离线运行链；模型“best”名称不代表全部指标占优。</footer></main></html>''')
(O/'report.html').write_text(''.join(parts))
print(O/'report.html')
