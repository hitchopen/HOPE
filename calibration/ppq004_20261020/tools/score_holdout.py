"""Recompute causal PPQ004 holdout predictions with the archived train-only fit."""
import argparse, json
import numpy as np
from refit_ppq_multibounce import position_contacts
from refit_c3d_recording import kernel, causal, summary
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument("--fit", type=Path, required=True);a=p.parse_args()
fit=a.fit
arcs=[];offset=0
for part in (1,2,3):
    data=dict(np.load(fit/f"part{part}/raw_extracted.npz"))
    found,_,_=position_contacts(data,part,offset,fit)
    arcs.extend(found);offset+=len(data["t"])/300
train=[x for x in arcs if x["part"]!=3];test=[x for x in arcs if x["part"]==3]
assert (len(train),len(test))==(246,27)
params=json.loads((fit/"training_fit.json").read_text())["params"]
rows=causal(test,kernel(fit),params)
assert len(rows)==24
result=summary(rows)
print(json.dumps({"train_arcs":len(train),"holdout_arcs":len(test),"causal_arcs":len(rows),"summary":result},indent=2))
