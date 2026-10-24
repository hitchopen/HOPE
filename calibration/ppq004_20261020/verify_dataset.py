"""Verify all nine compressed archives and optionally unpack into a new output tree."""
import argparse, gzip, hashlib, json, shutil
from pathlib import Path
root = Path(__file__).resolve().parent
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--unpack", type=Path)
a = p.parse_args()
if a.unpack:
    a.unpack.mkdir(parents=True, exist_ok=False)
for entry in json.loads((root / "manifest.json").read_text()):
    packed = (root / entry["path"]).read_bytes()
    assert hashlib.sha256(packed).hexdigest() == entry["compressed_sha256"], entry["path"]
    raw = gzip.decompress(packed)
    assert len(raw) == entry["bytes"] and hashlib.sha256(raw).hexdigest() == entry["sha256"], entry["path"]
    if a.unpack:
        target = a.unpack / entry["path"][:-3]
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(raw)
    print(entry["path"], "verified", len(raw))
if a.unpack:
    for file in (root / "fit").rglob("*.json"):
        target = a.unpack / file.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(file, target)
