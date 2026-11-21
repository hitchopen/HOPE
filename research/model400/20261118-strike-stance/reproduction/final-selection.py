from pathlib import Path
import hashlib,json,sys,time
w=Path(__file__).resolve().parent
stage=sys.argv[1];r=w/stage;d=json.loads((r/'score.json').read_text());b=d['v5']
groups=json.loads((r/'run.json').read_text())['policy_groups']
if stage=='strike-verify':
    hold=w/'hold-verify'
    while not (hold/'score.json').exists():time.sleep(2)
    hd=json.loads((hold/'score.json').read_text())
    assert hd['v5']==d['v5']
    d.update(hd);groups+=json.loads((hold/'run.json').read_text())['policy_groups'][1:]
def rank(k):
    a=d[k];s=a['all'];base=b['all']
    return ((s['legal']-base['legal'])/s['n']*100+(s['contacts']-base['contacts'])/s['n']*20+
            1.3*(b['mean_end_drift_cm']-a['mean_end_drift_cm'])+
            .35*(b['max_end_heading_deg']-a['max_end_heading_deg'])+
            100*(s['completed']-base['completed'])/s['n']+120*(base['collisions']-s['collisions'])/s['n']-
            sum(s[m][j]/base[m][j]-1 for m in ('position_cm','velocity_mps','normal_deg') for j in [0,1]))
order=sorted((k for k in d if k!='v5'),key=rank,reverse=True);print([(k,rank(k)) for k in order],flush=True)
if stage=='strike-screen':
    best=[order[0],next(k for k in order if k.startswith('center')!=order[0].startswith('center'))]
else:best=[order[0]]
final=[g for g in groups if g['label'] in ['v5',*best]]
(r/'selected.json').write_text(json.dumps(final,indent=2))
(r/'frozen.json').write_text(json.dumps({g['label']:hashlib.sha256((Path(g['path'])/'exported/policy.onnx').read_bytes()).hexdigest() for g in final},indent=2))
