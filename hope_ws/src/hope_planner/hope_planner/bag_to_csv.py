"""Convert a rosbag2 topic stream into HOPE calibration CSV (t,x,y,z).

This helper is intentionally small and practical:

- Input: a rosbag2 directory recorded from `/ball/point`
- Output: one CSV with columns `t,x,y,z`
- Time source: message header stamp if present, otherwise bag receive time
- Storage plugin: auto-detect `mcap` vs `sqlite3`
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import re

from geometry_msgs.msg import PointStamped
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def _detect_storage_id(bag_dir: Path) -> str:
    metadata = bag_dir / "metadata.yaml"
    if metadata.is_file():
        text = metadata.read_text(errors="replace")
        match = re.search(r"storage_identifier:\s*([A-Za-z0-9_]+)", text)
        if match:
            return match.group(1)

    mcap_files = list(bag_dir.glob("*.mcap"))
    db3_files = list(bag_dir.glob("*.db3"))
    if mcap_files:
        return "mcap"
    if db3_files:
        return "sqlite3"
    raise RuntimeError(
        f"Could not detect rosbag storage type in {bag_dir}. "
        "Expected metadata.yaml, *.mcap, or *.db3."
    )


def _check_bag_files_look_real(bag_dir: Path) -> None:
    data_files = list(bag_dir.glob("*.mcap")) + list(bag_dir.glob("*.db3"))
    if not data_files:
        raise RuntimeError(
            f"No bag data file found in {bag_dir}. "
            "The recording may not have finished correctly."
        )

    zero_files = [p.name for p in data_files if p.stat().st_size == 0]
    if zero_files:
        raise RuntimeError(
            f"Bag data file is empty in {bag_dir}: {', '.join(zero_files)}. "
            "This usually means recording did not finish correctly. Re-record this toss."
        )


def bag_point_topic_to_csv(bag_path: str, topic: str, output_csv: str) -> int:
    bag_dir = Path(bag_path)
    if not bag_dir.exists():
        raise RuntimeError(f"Bag path does not exist: {bag_path}")
    if not bag_dir.is_dir():
        raise RuntimeError(f"Bag path is not a directory: {bag_path}")

    _check_bag_files_look_real(bag_dir)
    storage_id = _detect_storage_id(bag_dir)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {meta.name: meta.type for meta in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise RuntimeError(f"Topic {topic!r} not found in bag {bag_path!r}")

    msg_type = get_message(topic_types[topic])
    rows = []

    while reader.has_next():
        current_topic, data, bag_time_ns = reader.read_next()
        if current_topic != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, PointStamped):
            raise RuntimeError(f"Topic {topic!r} is {topic_types[topic]!r}, expected PointStamped")

        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            t = bag_time_ns * 1e-9

        rows.append((t, msg.point.x, msg.point.y, msg.point.z))

    if not rows:
        raise RuntimeError(f"No messages found on topic {topic!r} in bag {bag_path!r}")

    rows.sort(key=lambda r: r[0])
    out_path = Path(output_csv)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "x", "y", "z"])
        writer.writerows(rows)
    return len(rows)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Convert a rosbag2 /ball/point stream into HOPE calibration CSV (t,x,y,z)."
    )
    parser.add_argument("--bag", required=True, help="Path to rosbag2 directory.")
    parser.add_argument(
        "--topic",
        default="/ball/point",
        help="PointStamped topic to export. Default: /ball/point",
    )
    parser.add_argument("--output", required=True, help="Output CSV path.")
    args = parser.parse_args(argv)

    count = bag_point_topic_to_csv(args.bag, args.topic, args.output)
    print(f"Wrote {count} rows to {args.output}")


if __name__ == "__main__":
    main()
