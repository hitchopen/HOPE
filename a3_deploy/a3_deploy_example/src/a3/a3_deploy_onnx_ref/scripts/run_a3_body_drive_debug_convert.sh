#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'USAGE'
Usage:
  ./run_a3_body_drive_debug_convert.sh bags/a3_body_drive_debug/<timestamp>/raw
  ./run_a3_body_drive_debug_convert.sh bags/a3_body_drive_debug/<timestamp>/raw --no-raw --compression none
  ./run_a3_body_drive_debug_convert.sh bags/a3_body_drive_debug/<timestamp>/raw --asset-dir /path/to/urdf/a3
  ./run_a3_body_drive_debug_convert.sh bags/a3_body_drive_debug/<timestamp>/raw --asset-dir /path/to/urdf/a3 --no-raw --mesh-mode data-uri
  ./run_a3_body_drive_debug_convert.sh bags/a3_body_drive_debug/<timestamp>/raw --asset-dir /path/to/urdf/a3 --no-raw --mesh-url-base http://127.0.0.1:8765/assets/a3

Converts AimRT raw body-drive MCAP files into a Foxglove-friendly MCAP plus
layout/settings files.

The deploy package does not bundle robot URDF/mesh assets. Pass --asset-dir
when you want the optional Foxglove 3D robot model layer.

When --asset-dir is provided, mesh files are embedded as MCAP attachments and
referenced from the URDF with package:// URLs, so Foxglove Desktop can open the
result without a separate asset server.

If your Foxglove Desktop cannot load package:// assets from MCAP attachments,
use --mesh-mode data-uri. It embeds the visual meshes directly in
/robot_description and writes that large message only once by default.

Large recordings convert much faster with --no-raw because it skips copying
the original 1000Hz topics and keeps only the derived Foxglove topics.

For browser/cloud Foxglove, --mesh-url-base can still be used instead of MCAP
attachments; serve the output directory with:
  python3 tools/serve_foxglove_assets.py <foxglove_output_dir> --host 127.0.0.1 --port 8765
USAGE
  exit 0
fi

PY_SCRIPT="${SCRIPT_DIR}/tools/a3_body_drive_debug_convert.py"
if [[ ! -f "${PY_SCRIPT}" ]]; then
  echo "missing converter: ${PY_SCRIPT}" >&2
  exit 66
fi

exec python3 "${PY_SCRIPT}" "$@"
