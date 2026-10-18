#!/usr/bin/python3
"""The robot may invoke only the existing Laptop helper, never a Mac shell."""
import os
import shlex
import sys

helper = "/home/dongc1/.local/libexec/hope-lifecycle"
allowed = {
    "preflight-laptop", "prepare-laptop", "start-laptop", "stop-laptop",
    "stop-capture-laptop", "collect", "field-assets-laptop", "record-control",
}
try:
    args = shlex.split(os.environ.get("SSH_ORIGINAL_COMMAND", ""))
except ValueError:
    sys.exit(64)
if len(args) < 2 or args[0] != helper or args[1] not in allowed:
    print("Only the fixed HOPE Laptop lifecycle actions are accepted.", file=sys.stderr)
    sys.exit(64)
os.execv(helper, args)
