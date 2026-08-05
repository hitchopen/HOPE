#!/usr/bin/env python3
"""Validate VRPN PoseStamped epochs and age against local system time.

This proves that the published header is a plausible, monotonic Unix/NTP
timestamp. It cannot prove which physical camera event CMTracker associates
with the proprietary VRPN server timestamp; that requires a vendor/SDK or
hardware-trigger comparison.
"""

from __future__ import annotations

import argparse
import os
import re
import statistics
import subprocess
import time


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--samples", type=int, default=300)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--max-age-ms", type=float, default=100.0)
    parser.add_argument("--max-p95-age-ms", type=float, default=20.0)
    parser.add_argument("--max-future-ms", type=float, default=5.0)
    parser.add_argument("--min-hz", type=float, default=100.0)
    parser.add_argument("--max-ntp-offset-ms", type=float, default=1.0)
    parser.add_argument(
        "--skip-ntp-check",
        action="store_true",
        help="Skip the local chronyc health gate (non-Chrony platforms only)",
    )
    return parser.parse_args()


def _chrony_health(max_offset_ms: float) -> tuple[bool, str]:
    """Check the adapter host's system clock against its Chrony source."""
    try:
        result = subprocess.run(
            ["chronyc", "tracking"],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
            env={**os.environ, "LC_ALL": "C"},
        )
    except (FileNotFoundError, subprocess.SubprocessError) as error:
        return False, f"chronyc unavailable: {error}"

    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        return False, f"chronyc tracking failed: {detail}"

    system_time = re.search(
        r"^System time\s*:\s*([0-9.eE+-]+) seconds (fast|slow)",
        result.stdout,
        re.MULTILINE,
    )
    leap_status = re.search(
        r"^Leap status\s*:\s*(\S+)", result.stdout, re.MULTILINE
    )
    stratum = re.search(r"^Stratum\s*:\s*(\d+)", result.stdout, re.MULTILINE)
    if system_time is None or leap_status is None or stratum is None:
        return False, "could not parse chronyc tracking output"

    offset_ms = abs(float(system_time.group(1))) * 1000.0
    leap = leap_status.group(1)
    stratum_value = int(stratum.group(1))
    healthy = (
        leap.lower() == "normal"
        and 1 <= stratum_value < 16
        and offset_ms <= max_offset_ms
    )
    return healthy, (
        f"system_offset_ms={offset_ms:.6f}, leap={leap}, "
        f"stratum={stratum_value}, limit_ms={max_offset_ms:.6f}"
    )


def main() -> None:
    args = _args()
    if (
        args.samples < 2
        or args.timeout_s <= 0.0
        or args.max_age_ms < 0.0
        or args.max_p95_age_ms < 0.0
        or args.max_p95_age_ms > args.max_age_ms
        or args.max_future_ms < 0.0
        or args.min_hz <= 0.0
        or args.max_ntp_offset_ms < 0.0
    ):
        raise SystemExit("invalid non-positive sample/rate/time bound")

    if args.skip_ntp_check:
        print("ntp_health=SKIPPED (operator must verify the system clock externally)")
    else:
        ntp_healthy, ntp_detail = _chrony_health(args.max_ntp_offset_ms)
        print(f"ntp_health={'PASS' if ntp_healthy else 'FAIL'}: {ntp_detail}")
        if not ntp_healthy:
            raise SystemExit("FAIL: adapter host is not demonstrably NTP disciplined")

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("vrpn_timestamp_probe")
    stamps_ns: list[int] = []
    ages_ms: list[float] = []

    def callback(message: PoseStamped) -> None:
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        stamps_ns.append(stamp_ns)
        ages_ms.append((time.time_ns() - stamp_ns) / 1_000_000.0)

    subscription = node.create_subscription(
        PoseStamped, args.topic, callback, qos_profile_sensor_data
    )

    deadline = time.monotonic() + args.timeout_s
    try:
        while len(stamps_ns) < args.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

    if len(stamps_ns) < args.samples:
        raise SystemExit(
            f"FAIL: received {len(stamps_ns)}/{args.samples} samples on {args.topic}"
        )

    strictly_increasing = all(
        later > earlier for earlier, later in zip(stamps_ns, stamps_ns[1:])
    )
    duration_s = (stamps_ns[-1] - stamps_ns[0]) * 1e-9
    source_rate_hz = (len(stamps_ns) - 1) / duration_s if duration_s > 0.0 else 0.0
    ordered_ages = sorted(ages_ms)
    p95_age_ms = ordered_ages[int(0.95 * (len(ordered_ages) - 1))]
    minimum_age_ms = min(ages_ms)
    maximum_age_ms = max(ages_ms)

    print(f"topic={args.topic}")
    print(f"samples={len(stamps_ns)}")
    print(f"source_rate_hz={source_rate_hz:.3f}")
    print(f"age_ms_min={minimum_age_ms:.3f}")
    print(f"age_ms_median={statistics.median(ages_ms):.3f}")
    print(f"age_ms_p95={p95_age_ms:.3f}")
    print(f"age_ms_max={maximum_age_ms:.3f}")
    print(f"strictly_increasing={str(strictly_increasing).lower()}")
    print("camera_exposure_provenance=not_inferable_from_vrpn_packet")

    passed = (
        strictly_increasing
        and source_rate_hz >= args.min_hz
        and minimum_age_ms >= -args.max_future_ms
        and p95_age_ms <= args.max_p95_age_ms
        and maximum_age_ms <= args.max_age_ms
    )
    if not passed:
        raise SystemExit("FAIL: VRPN timestamp acceptance gate failed")
    print("PASS: VRPN timestamps share the adapter's validated NTP/Unix epoch")


if __name__ == "__main__":
    main()
