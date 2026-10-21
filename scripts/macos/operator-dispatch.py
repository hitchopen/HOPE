#!/usr/bin/python3
"""Map the trusted HDU Laptop helper contract to the container's fixed helper."""
import sys
if __name__ == "__main__" and sys.path:
    sys.path.pop(0)  # Do not import the sibling operator.py as Python's stdlib.

import json
import os
from pathlib import Path
import pwd
import re
import shlex
import stat
import tempfile

CONFIG_PATH = Path("/etc/hope-mac-operator.json")
ALLOWED_ACTIONS = frozenset({
    "preflight-laptop", "prepare-laptop", "start-laptop", "stop-laptop",
    "stop-capture-laptop", "collect", "field-assets-laptop", "record-control",
})


def validate_contract(user, helper):
    if (not isinstance(user, str) or user == "root" or
            re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", user) is None):
        raise ValueError("HOPE_OPERATOR_USER must be a non-root POSIX account name")
    if (not isinstance(helper, str) or helper == "/" or helper.endswith("/") or
            re.fullmatch(r"/[A-Za-z0-9_./-]+", helper) is None or
            any(part in {".", ".."} for part in helper.split("/"))):
        raise ValueError("HOPE_OPERATOR_HELPER must be a safe absolute executable path")
    return {"laptop_user": user, "laptop_helper": helper}


def configure_from_environment():
    """Container initialization only; never available to the SSH login account."""
    if os.geteuid() != 0:
        raise ValueError("operator configuration requires root")
    contract = validate_contract(os.environ.get("HOPE_OPERATOR_USER"),
                                 os.environ.get("HOPE_OPERATOR_HELPER"))
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=CONFIG_PATH.parent,
                                         prefix=".hope-mac-operator-", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(contract, stream)
            stream.write("\n")
            stream.flush()
            os.fchmod(stream.fileno(), 0o644)
            os.fchown(stream.fileno(), 0, 0)
        os.replace(temporary, CONFIG_PATH)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def read_contract():
    # The login account must not be able to substitute the trusted contract.
    descriptor = os.open(CONFIG_PATH, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(descriptor, "r", encoding="utf-8") as stream:
        metadata = os.fstat(stream.fileno())
        if (not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or
                metadata.st_mode & 0o022):
            raise ValueError("operator configuration must be a root-owned, non-writable regular file")
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError("invalid operator configuration")
    return validate_contract(document.get("laptop_user"), document.get("laptop_helper"))


def dispatch(command, contract):
    contract = validate_contract(contract.get("laptop_user"), contract.get("laptop_helper"))
    account = pwd.getpwuid(os.geteuid())
    if account.pw_uid < 1000 or account.pw_name != contract["laptop_user"]:
        raise ValueError("SSH login does not match the configured operator account")
    args = shlex.split(command)
    if (len(args) < 2 or args[0] != contract["laptop_helper"] or
            args[1] not in ALLOWED_ACTIONS):
        raise ValueError("Only the configured HOPE Laptop helper and fixed actions are accepted")
    # Never execute a path supplied by the remote command or external contract.
    helper = f"/home/{contract['laptop_user']}/.local/libexec/hope-lifecycle"
    os.execv(helper, [helper, *args[1:]])


def main():
    try:
        if sys.argv[1:] == ["--configure"]:
            configure_from_environment()
        elif sys.argv[1:]:
            raise ValueError("unexpected dispatcher arguments")
        else:
            dispatch(os.environ.get("SSH_ORIGINAL_COMMAND", ""), read_contract())
    except (OSError, ValueError, KeyError) as exc:
        print(str(exc), file=sys.stderr)
        return 64
    return 0


if __name__ == "__main__":
    sys.exit(main())
