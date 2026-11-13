from build_super_factorial import solve,W
import json,concurrent.futures
requested=json.loads((W/'super-factorial-requested.json').read_text());selected=json.loads((W/'super-factorial-selected.json').read_text())
cell=(0,0,0,2);specs=[(*cell,v) for v in range(72,180)]
with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool:
 for row in pool.map(solve,specs):
  requested.append(row)
  if row['feasible'] and len(selected)<972:selected.append(row);print('selected',len(selected),flush=True)
(W/'super-factorial-requested.json').write_text(json.dumps(requested,indent=2));(W/'super-factorial-selected.json').write_text(json.dumps(selected,indent=2));assert len(selected)==972,len(selected)
