#!/usr/bin/env python3
"""Read-only cable, four-direction SSH and Foxglove handshake checks.

This never starts a Runner, calls a ROS service or changes SSH host trust.
Pass the HDU address actually observed on this robot, not an old runbook IP.
"""

import argparse
import base64
import getpass
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import socket
import subprocess
import sys


def run(argv, timeout=25):
    result = subprocess.run(argv, text=True, capture_output=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError((result.stderr or result.stdout).strip())
    return result.stdout.strip()


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def route(host):
    return json.loads(run(["ip", "-j", "-4", "route", "get", host]))[0]


def handshake(host, port, source):
    key = base64.b64encode(os.urandom(16)).decode()
    request = (
        f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
        "Sec-WebSocket-Protocol: foxglove.sdk.v1, foxglove.websocket.v1\r\n\r\n"
    )
    with socket.create_connection((host, port), timeout=5,
                                  source_address=(source, 0)) as sock:
        sock.sendall(request.encode())
        data = bytearray()
        while b"\r\n\r\n" not in data:
            chunk = sock.recv(4096)
            require(chunk, "WebSocket closed before HTTP upgrade")
            data.extend(chunk)
            require(len(data) <= 65536, "Oversized HTTP upgrade response")
    lines = data.split(b"\r\n\r\n", 1)[0].decode("ascii").split("\r\n")
    status = lines[0].split()
    require(len(status) >= 2 and status[1] == "101", lines[0])
    headers = {k.strip().lower(): v.strip() for k, v in
               (line.split(":", 1) for line in lines[1:])}
    accept = base64.b64encode(hashlib.sha1(
        (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
    require(headers.get("sec-websocket-accept") == accept, "Invalid WebSocket accept")
    protocol = headers.get("sec-websocket-protocol")
    require(protocol in {"foxglove.sdk.v1", "foxglove.websocket.v1"},
            "Endpoint did not select the Foxglove protocol")
    return protocol


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--laptop", required=True, type=ipaddress.IPv4Address)
    parser.add_argument("--hdu", required=True, type=ipaddress.IPv4Address)
    parser.add_argument("--mdu", default="10.42.10.12", type=ipaddress.IPv4Address)
    parser.add_argument("--laptop-user", default=getpass.getuser())
    parser.add_argument("--robot-user", default="agi")
    args = parser.parse_args()
    for name in (args.laptop_user, args.robot_user):
        require(re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", name), "Invalid SSH user")
    require(re.fullmatch(r"[A-Za-z0-9_.:-]+", args.interface), "Invalid interface")
    laptop, hdu, mdu = map(str, (args.laptop, args.hdu, args.mdu))
    require(Path(f"/sys/class/net/{args.interface}/carrier").read_text().strip() == "1",
            f"{args.interface}: NO-CARRIER; check cable, port and HDU power")
    r = route(hdu)
    require(r.get("dev") == args.interface and r.get("prefsrc") == laptop
            and "gateway" not in r, f"Not the expected direct cable route: {r}")
    print(f"PASS cable route: {laptop} -> {hdu} via {args.interface}", flush=True)
    options = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
               "-o", "HostKeyAlgorithms=ssh-ed25519", "-o", "ConnectTimeout=5",
               "-o", "ConnectionAttempts=1", "-o", "ServerAliveInterval=5",
               "-o", "ServerAliveCountMax=1"]
    # Explicit -J replaces a legacy Wi-Fi ProxyJump for the MDU.
    hdu_ssh = ["ssh", *options, f"{args.robot_user}@{hdu}"]
    mdu_ssh = ["ssh", *options, "-J", f"{args.robot_user}@{hdu}",
               f"{args.robot_user}@{mdu}"]
    checks = [
        ("Laptop -> HDU", [*hdu_ssh, "hostname -s"], "hdu"),
        ("Laptop -> MDU via HDU", [*mdu_ssh, "hostname -s"], "mdu"),
        ("HDU -> Laptop", [*hdu_ssh, shlex.join([
            "ssh", *options, f"{args.laptop_user}@{laptop}", "hostname -s"])],
         socket.gethostname().split(".")[0]),
        ("HDU -> MDU", [*hdu_ssh, shlex.join([
            "ssh", *options, f"{args.robot_user}@{mdu}", "hostname -s"])], "mdu"),
    ]
    failures = []
    for label, command, expected in checks:
        try:
            actual = run(command)
            require(actual == expected, f"Expected {expected}, got {actual!r}")
            print(f"PASS {label}: {actual}", flush=True)
        except (RuntimeError, subprocess.TimeoutExpired) as exc:
            failures.append(label)
            print(f"FAIL {label}: {exc}", flush=True)
    require(not failures, "SSH checks failed: " + ", ".join(failures))
    print("FOUR_SSH_DIRECTIONS_OK", flush=True)
    back = json.loads(run([*hdu_ssh, f"ip -j -4 route get {laptop}"]))[0]
    require(back.get("prefsrc") == hdu and "gateway" not in back,
            f"HDU return route does not use its direct address: {back}")
    print(f"PASS HDU return route: {back}", flush=True)
    for port in (8766, 8765):
        protocol = handshake(hdu, port, laptop)
        print(f"PASS Foxglove protocol: ws://{hdu}:{port} ({protocol})", flush=True)
    print("WIRED_SSH_AND_WEBSOCKET_OK", flush=True)
    print("Network check only; lifecycle configuration, live ROS data and clock gates "
          "must be verified separately before operating the robot.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
