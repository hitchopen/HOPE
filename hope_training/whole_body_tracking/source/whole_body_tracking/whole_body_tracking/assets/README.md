# Generated Assets

This package exists so `whole_body_tracking.assets.ASSET_DIR` is importable from
a fresh clone.

Generate the local A3 Isaac asset with (from the repo root):

```bash
python3 hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py --force
```

The generated `agibot_a3/` directory is derived from the Agibot-provided source
URDF package — `agibot/URDF/A3T2.5-URDF-std-pingpang/` when present (the
default), or your own copy under `a3_deploy/URDF/` via `--source-root` (see
`a3_deploy/URDF/README.md`) — and is intentionally git-ignored.
