#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
APP_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG="${1:-${APP_ROOT}/config/serve.json}"
OUTPUT="${2:-${APP_ROOT}/build/generated/default}"

export PYTHONPATH="${APP_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
exec python3 -m a3_serve.cli generate --config "${CONFIG}" --output "${OUTPUT}"

