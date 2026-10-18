#!/bin/bash
set -euo pipefail
export PATH="$HOME/.docker/bin:/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin"
python3 - <<'PY'
import json
from pathlib import Path
import subprocess
import sys
from urllib.parse import urlencode
cfg=json.loads((Path.home()/'.config/hope-macos/operator.json').read_text())
subprocess.run([sys.executable,str(Path(cfg['root'])/'scripts/macos/operator.py'),'up'],check=True)
url='foxglove://open?'+urlencode({'ds':'foxglove-websocket','ds.url':f"ws://{cfg['hdu_ip']}:8766",'layoutId':cfg['layout_id']})
subprocess.run(['/usr/bin/open','-a','/Applications/Foxglove.app',url],check=True)
PY
