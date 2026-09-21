#!/usr/bin/env python3
"""Fixed-path, ROS-free field asset storage. JSON requests arrive on stdin.

The laptop owns the active calibration; the MDU owns immutable serve CSVs.
No request can select an executable or a destination outside these stores.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

MAX_CALIBRATION_BYTES = 1024 * 1024
MAX_CSV_BYTES = 8 * 1024 * 1024


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".upload-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        Path(temporary).unlink(missing_ok=True)


def asset_name(value: object, suffix: str) -> str:
    if not isinstance(value, str) or not value or len(value) > 180:
        raise ValueError("provide a filename of at most 180 characters")
    if Path(value).name != value or "\\" in value or any(ord(c) < 32 for c in value):
        raise ValueError("filename must not contain a path or control characters")
    if not value.lower().endswith(suffix):
        raise ValueError(f"filename must end with {suffix}")
    return value


def calibration_metadata(content: str, name: str, side: str) -> dict:
    # Same validator as the live base relay; deployment installs both modules.
    from p1_calibration import decode_p1_calibration

    encoded = content.encode("utf-8")
    if len(encoded) > MAX_CALIBRATION_BYTES:
        raise ValueError("calibration exceeds 1 MiB")
    receipt = decode_p1_calibration(encoded)
    document = json.loads(content)
    if side not in {"P1", "P2"}:
        raise ValueError("table side must be P1 or P2")
    if receipt.parent_frame != f"UCB_{side}" or receipt.child_frame != "pelvis_link":
        raise ValueError(f"calibration must describe UCB_{side} -> pelvis_link")
    if receipt.rigid_body_id is None:
        raise ValueError("calibration is missing its Motive rigid body identity")
    profile = document.get("cad", {}).get("marker_layout")
    if profile is None and document.get("marker_layout"):
        profile = "stickers_v3"
    if profile not in {"v2", "v3", "stickers_v3"}:
        raise ValueError("calibration must identify marker layout stickers_v3, v2 or v3")
    if document.get("runtime_world", {}).get("table_side", "P1" if profile == "stickers_v3" else None) != side:
        raise ValueError("calibration runtime world does not match the selected table side")
    return dict(name=asset_name(name, ".json"), sha256=receipt.receipt_sha256,
                profile=profile, table_side=side, rigid_body_id=receipt.rigid_body_id)


def calibration_request(root: Path, request: dict) -> dict:
    active = root / "ucb_robot_to_pelvis.json"
    operation = request.get("op")
    side = request.get("table_side", "P1")
    if operation == "load_calibration":
        content = request["content"]
        metadata = calibration_metadata(content, request["name"], side)
        # Keep every approved venue receipt even after selecting another one.
        atomic_write(root / "venues" / (metadata["sha256"] + ".json"), content.encode())
        atomic_write(active, content.encode())
        atomic_write(root / "active_field_asset.json", json.dumps(metadata).encode())
    elif operation != "read_calibration":
        raise ValueError("unsupported laptop asset operation")
    if not active.exists():
        return {"calibration": None}
    content = active.read_text(encoding="utf-8")
    metadata = calibration_metadata(content, active.name, side)
    try:
        label = json.loads((root / "active_field_asset.json").read_text())
        if label["sha256"] == metadata["sha256"]:
            metadata["name"] = asset_name(label["name"], ".json")
    except (OSError, ValueError, KeyError):
        pass
    return {"calibration": metadata, "content": content}


def csv_request(root: Path, validator: Path, request: dict, *, run=subprocess.run) -> dict:
    operation = request.get("op")
    if operation == "verify_csv":
        digest = str(request.get("sha256", ""))
        if re.fullmatch(r"[0-9a-f]{64}", digest) is None:
            raise ValueError("invalid CSV hash")
        target = root / (digest + ".csv")
        if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            raise ValueError("selected CSV changed on the MDU; upload it again")
    elif operation == "upload_csv":
        asset_name(request["name"], ".csv")
        encoded = request["content"].encode("utf-8")
        if not encoded or len(encoded) > MAX_CSV_BYTES:
            raise ValueError("CSV must contain 1 byte to 8 MiB")
        digest = hashlib.sha256(encoded).hexdigest()
        root.mkdir(parents=True, exist_ok=True)
        fd, staging = tempfile.mkstemp(prefix=".validate-", suffix=".csv", dir=root)
        try:
            with os.fdopen(fd, "wb") as stream:
                stream.write(encoded)
            result = run([str(validator), "--validate-serve-timeline", staging],
                         capture_output=True, text=True, timeout=20, check=False)
            if result.returncode:
                raise ValueError("CSV rejected: " + (result.stderr or result.stdout)[-1500:])
            # Require the new validation-only binary, not an old binary ignoring flags.
            if "SERVE_TIMELINE_VALID" not in result.stdout:
                raise ValueError("MDU Runner needs the field-workflow update")
            target = root / (digest + ".csv")
            atomic_write(target, encoded)
        finally:
            Path(staging).unlink(missing_ok=True)
    else:
        raise ValueError("unsupported MDU asset operation")
    return {"sha256": digest, "path": str(target), "frames": 468, "hz": 100}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("host", choices=("laptop", "mdu"))
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--validator", type=Path)
    args = parser.parse_args()
    try:
        raw = sys.stdin.buffer.read(MAX_CSV_BYTES + MAX_CALIBRATION_BYTES + 1)
        if len(raw) > MAX_CSV_BYTES + MAX_CALIBRATION_BYTES:
            raise ValueError("request too large")
        request = json.loads(raw)
        if args.host == "laptop":
            result = calibration_request(args.root, request)
        else:
            if args.validator is None:
                raise ValueError("validator required")
            result = csv_request(args.root, args.validator, request)
        print(json.dumps(result, ensure_ascii=False))
    except (OSError, ValueError, KeyError, TypeError, AttributeError, subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
