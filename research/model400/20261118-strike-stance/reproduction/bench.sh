set -eu
w="$(cd "$(dirname "$0")" && pwd)"
p=/home/user/Documents/Codex/2026-11-04/new-chat/work/venv/bin/python
exec distrobox enter hope -- env OPENBLAS_NUM_THREADS=1 PP_GATE3_SUPPORT_PROFILE="$w/../rally400/support.yaml" "$p" "$w/run.py" "$1" "$2" "$w/$3" --dataset "$4" --policy-groups "$w/$5"
