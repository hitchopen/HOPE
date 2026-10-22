"""Replay archived source-time PPQ002 trajectories through the current core."""
import argparse,csv,gzip,hashlib,io,json,subprocess,tempfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument("--executable",type=Path,required=True);a=p.parse_args()
root=Path(__file__).resolve().parents[1]/"test/data/deadline_20261020"
results=[]
with tempfile.TemporaryDirectory() as tmp:
    for row in json.loads((root/"expected.json").read_text()):
        data=gzip.decompress((root/row["input"]).read_bytes())
        assert hashlib.sha256(data).hexdigest()==row["input_sha256"]
        f=Path(tmp)/"trajectory.csv";f.write_bytes(data)
        run=subprocess.run([str(a.executable.resolve()),str(f),str(row["x_hit"])],capture_output=True,text=True,check=True)
        current=list(csv.DictReader(io.StringIO(run.stdout)))
        assert len(current)==1,(row["flight"],run.stdout)
        expected=row["result"][0]
        for key in ("commit","strike","lead","x","y","z"):
            assert abs(float(current[0][key])-float(expected[key]))<1e-5,(row["flight"],key,current[0][key],expected[key])
        results.append(float(current[0]["lead"]))
assert len(results)==27
print(json.dumps({"trajectories":len(results),"packets":len(results),"lead_400_405ms":sum(.399<=v<=.406 for v in results),"minimum_lead_s":min(results)}))
