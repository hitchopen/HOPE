#!/usr/bin/env python3
"""Convert A3 body-drive raw AimRT MCAP files into a Foxglove-friendly MCAP."""

from __future__ import annotations

import argparse
import base64
import json
import re
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


A3_JOINT_NAMES_31 = [
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

URDF_JOINT_NAMES = [name for name in A3_JOINT_NAMES_31 if not name.startswith("head_")]
A3_INDEX_BY_NAME = {name: index for index, name in enumerate(A3_JOINT_NAMES_31)}

GROUPS = {
    "waist": {
        "state_topic": "/body_drive/waist_joint_state",
        "command_topic": "/body_drive/waist_joint_command",
        "start": 0,
        "names": A3_JOINT_NAMES_31[0:3],
    },
    "neck": {
        "state_topic": "/body_drive/neck_joint_state",
        "command_topic": "/body_drive/neck_joint_command",
        "start": 3,
        "names": A3_JOINT_NAMES_31[3:5],
    },
    "arm": {
        "state_topic": "/body_drive/arm_joint_state",
        "command_topic": "/body_drive/arm_joint_command",
        "start": 5,
        "names": A3_JOINT_NAMES_31[5:19],
    },
    "leg": {
        "state_topic": "/body_drive/leg_joint_state",
        "command_topic": "/body_drive/leg_joint_command",
        "start": 19,
        "names": A3_JOINT_NAMES_31[19:31],
    },
}

STATE_TOPIC_TO_GROUP = {cfg["state_topic"]: group for group, cfg in GROUPS.items()}
COMMAND_TOPIC_TO_GROUP = {cfg["command_topic"]: group for group, cfg in GROUPS.items()}

PANEL_TITLE_CONFIG_KEY = "foxglovePanelTitle"

PLOT_TAB_SPECS = [
    ("waist pos", "waist", "pos", "position", "q state", "q command", "position [rad]"),
    ("waist tau", "waist", "tau", "effort", "tau measured", "tau PD command", "torque [Nm]"),
    ("waist vel", "waist", "vel", "velocity", "dq state", "dq command", "velocity [rad/s]"),
    ("leg pos", "leg", "pos", "position", "q state", "q command", "position [rad]"),
    ("leg tau", "leg", "tau", "effort", "tau measured", "tau PD command", "torque [Nm]"),
    ("leg vel", "leg", "vel", "velocity", "dq state", "dq command", "velocity [rad/s]"),
    ("arm pos", "arm", "pos", "position", "q state", "q command", "position [rad]"),
    ("arm tau", "arm", "tau", "effort", "tau measured", "tau PD command", "torque [Nm]"),
    ("arm vel", "arm", "vel", "velocity", "dq state", "dq command", "velocity [rad/s]"),
]

ACTUAL_SERIES_COLOR = "#4E79A7"
COMMAND_SERIES_COLOR = "#F28E2B"


JOINT_MSG_DEFS = {
    "joint_msgs/msg/State": """\
string name
uint32 sequence
float64 position
float64 velocity
float64 effort
""",
    "joint_msgs/msg/Command": """\
string name
uint32 sequence
float64 position
float64 velocity
float64 effort
float64 stiffness
float64 damping
""",
    "joint_msgs/msg/JointState": """\
std_msgs/Header header
joint_msgs/State[] joints
""",
    "joint_msgs/msg/JointCommand": """\
std_msgs/Header header
joint_msgs/Command[] joints
""",
}


@dataclass
class Channels:
    robot_description: int
    tf: int
    joint_states: int
    actual: int
    desired: int
    error: int
    table: int
    timing: int


@dataclass(frozen=True)
class MeshAttachment:
    name: str
    path: Path
    media_type: str


def get_ros2_typestore():
    store = getattr(Stores, "ROS2_HUMBLE", None) or getattr(Stores, "ROS2_JAZZY", None)
    if store is None:
        store = Stores.LATEST
    typestore = get_typestore(store)
    custom_types = {}
    for name, text in JOINT_MSG_DEFS.items():
        custom_types.update(get_types_from_msg(text, name))
    typestore.register(custom_types)
    return typestore


def ns_to_time(typestore, timestamp_ns: int):
    time_cls = typestore.types["builtin_interfaces/msg/Time"]
    sec = int(timestamp_ns // 1_000_000_000)
    nanosec = int(timestamp_ns % 1_000_000_000)
    return time_cls(sec=sec, nanosec=nanosec)


def make_header(typestore, timestamp_ns: int, frame_id: str = ""):
    header_cls = typestore.types["std_msgs/msg/Header"]
    return header_cls(stamp=ns_to_time(typestore, timestamp_ns), frame_id=frame_id)


def msg_stamp_ns(msg: Any, fallback_ns: int) -> int:
    try:
        stamp = msg.header.stamp
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
        if sec != 0 or nanosec != 0:
            return sec * 1_000_000_000 + nanosec
    except Exception:
        pass
    return fallback_ns


def values_by_name(rows: list[Any], expected_names: list[str], fields: list[str]) -> list[list[float]]:
    latest: dict[str, Any] = {getattr(row, "name", ""): row for row in rows}
    out = []
    missing = [name for name in expected_names if name not in latest]
    if missing:
        raise ValueError(f"missing joint names: {', '.join(missing)}")
    for field in fields:
        out.append([float(getattr(latest[name], field)) for name in expected_names])
    return out


def find_mcap_files(raw_path: Path) -> list[Path]:
    if raw_path.is_file() and raw_path.suffix == ".mcap":
        return [raw_path]
    files = sorted(raw_path.rglob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap files found under {raw_path}")
    return files


def infer_session_root(raw_path: Path) -> Path:
    raw_path = raw_path.resolve()
    if raw_path.is_file():
        raw_dir = raw_path.parent
    else:
        raw_dir = raw_path
    if raw_dir.name.startswith("aimrtbag") and raw_dir.parent.name == "raw":
        return raw_dir.parent.parent
    if raw_dir.name == "raw":
        return raw_dir.parent
    return raw_dir.parent


def find_a3_asset_dir(script_path: Path, override: str | None) -> Path:
    del script_path
    if override:
        candidate = Path(override).expanduser()
        if (candidate / "model.urdf").is_file() and (candidate / "meshes").is_dir():
            return candidate.resolve()
    raise FileNotFoundError(
        "could not locate A3 URDF assets. The deploy package does not bundle "
        "URDF/mesh files; pass --asset-dir /path/to/urdf/a3 when Foxglove 3D "
        "robot model output is needed."
    )


def mesh_media_type(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix == ".stl":
        return "model/stl"
    if suffix == ".dae":
        return "model/vnd.collada+xml"
    if suffix == ".obj":
        return "model/obj"
    return "application/octet-stream"


def mesh_data_uri(path: Path) -> str:
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    # Keep the original suffix in the fragment; common URDF mesh loaders select
    # STL/DAE/OBJ loaders from the URL suffix before fetching the resource.
    return f"data:{mesh_media_type(path)};base64,{encoded}#{path.name}"


def remove_collision_nodes(root: ET.Element) -> None:
    for link in root.findall(".//link"):
        for child in list(link):
            if child.tag == "collision":
                link.remove(child)


def copy_and_rewrite_urdf(
    asset_dir: Path,
    output_assets_dir: Path,
    mesh_url_base: str | None = None,
    mesh_mode: str = "attachment",
) -> tuple[str, list[MeshAttachment]]:
    dst = output_assets_dir / "a3"
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(asset_dir, dst)

    urdf_path = dst / "model.urdf"
    text = urdf_path.read_text(encoding="utf-8")
    normalized_mesh_url_base = mesh_url_base.rstrip("/") if mesh_url_base else None
    attachments_by_name: dict[str, MeshAttachment] = {}

    if mesh_mode == "data-uri":
        root = ET.fromstring(text)
        remove_collision_nodes(root)
        for mesh in root.findall(".//mesh"):
            filename = mesh.attrib.get("filename", "")
            if not filename or filename.startswith(("file://", "package://", "http://", "https://", "/")):
                continue
            mesh.attrib["filename"] = mesh_data_uri((dst / filename).resolve())
        text = ET.tostring(root, encoding="unicode")
        foxglove_urdf = dst / "model.foxglove.urdf"
        foxglove_urdf.write_text(text, encoding="utf-8")
        return text, []

    def repl(match: re.Match[str]) -> str:
        filename = match.group(1)
        if normalized_mesh_url_base and not filename.startswith(("file://", "package://", "http://", "https://", "/")):
            return f'filename="{normalized_mesh_url_base}/{filename}"'
        if filename.startswith(("file://", "package://", "/")):
            return f'filename="{filename}"'
        mesh_path = (dst / filename).resolve()
        attachment_name = f"package://a3/{filename}"
        attachments_by_name.setdefault(
            attachment_name,
            MeshAttachment(name=attachment_name, path=mesh_path, media_type=mesh_media_type(mesh_path)),
        )
        return f'filename="{attachment_name}"'

    text = re.sub(r'filename="([^"]+)"', repl, text)
    foxglove_urdf = dst / "model.foxglove.urdf"
    foxglove_urdf.write_text(text, encoding="utf-8")
    return text, list(attachments_by_name.values())


def extract_urdf_joint_names(robot_description: str) -> list[str]:
    root = ET.fromstring(robot_description)
    names: list[str] = []
    seen: set[str] = set()
    for joint in root.findall(".//joint"):
        name = joint.attrib.get("name")
        if name and name not in seen:
            names.append(name)
            seen.add(name)
    return names or list(URDF_JOINT_NAMES)


def values_for_joint_names(names: list[str], position, velocity, effort):
    out_position = []
    out_velocity = []
    out_effort = []
    for name in names:
        index = A3_INDEX_BY_NAME.get(name)
        if index is None:
            out_position.append(0.0)
            out_velocity.append(0.0)
            out_effort.append(0.0)
            continue
        out_position.append(float(position[index]))
        out_velocity.append(float(velocity[index]))
        out_effort.append(float(effort[index]))
    return out_position, out_velocity, out_effort


def register_ros2_schema(writer: Writer, typestore, typename: str) -> int:
    msgdef, _ = typestore.generate_msgdef(typename, ros_version=2)
    return writer.register_schema(typename, "ros2msg", msgdef.encode("utf-8"))


def register_derived_channels(writer: Writer, typestore) -> Channels:
    string_schema = register_ros2_schema(writer, typestore, "std_msgs/msg/String")
    tf_schema = register_ros2_schema(writer, typestore, "tf2_msgs/msg/TFMessage")
    joint_schema = register_ros2_schema(writer, typestore, "sensor_msgs/msg/JointState")
    diag_schema = register_ros2_schema(writer, typestore, "diagnostic_msgs/msg/DiagnosticArray")
    return Channels(
        robot_description=writer.register_channel("/robot_description", "cdr", string_schema),
        tf=writer.register_channel("/tf", "cdr", tf_schema),
        joint_states=writer.register_channel("/joint_states", "cdr", joint_schema),
        actual=writer.register_channel("/a3_debug/actual_joint_states_31", "cdr", joint_schema),
        desired=writer.register_channel("/a3_debug/desired_joint_states_31", "cdr", joint_schema),
        error=writer.register_channel("/a3_debug/error_joint_states_31", "cdr", joint_schema),
        table=writer.register_channel("/a3_debug/joint_error_table", "cdr", diag_schema),
        timing=writer.register_channel("/a3_debug/timing", "cdr", diag_schema),
    )


def make_joint_state(typestore, timestamp_ns: int, names: list[str], position, velocity, effort):
    cls = typestore.types["sensor_msgs/msg/JointState"]
    return cls(
        header=make_header(typestore, timestamp_ns, "pelvis_link"),
        name=list(names),
        position=np.asarray(position, dtype=np.float64),
        velocity=np.asarray(velocity, dtype=np.float64),
        effort=np.asarray(effort, dtype=np.float64),
    )


def make_pelvis_tf_from_imu(typestore, timestamp_ns: int, imu_msg: Any):
    vector_cls = typestore.types["geometry_msgs/msg/Vector3"]
    transform_cls = typestore.types["geometry_msgs/msg/Transform"]
    stamped_cls = typestore.types["geometry_msgs/msg/TransformStamped"]
    tf_cls = typestore.types["tf2_msgs/msg/TFMessage"]
    return tf_cls(
        transforms=[
            stamped_cls(
                header=make_header(typestore, timestamp_ns, "world"),
                child_frame_id="pelvis_link",
                transform=transform_cls(
                    translation=vector_cls(x=0.0, y=0.0, z=0.0),
                    rotation=imu_msg.orientation,
                ),
            )
        ]
    )


def make_diag(typestore, timestamp_ns: int, statuses: list[Any]):
    cls = typestore.types["diagnostic_msgs/msg/DiagnosticArray"]
    return cls(header=make_header(typestore, timestamp_ns, "pelvis_link"), status=statuses)


def key_values(typestore, pairs: list[tuple[str, Any]]) -> list[Any]:
    cls = typestore.types["diagnostic_msgs/msg/KeyValue"]
    return [cls(key=str(k), value=str(v)) for k, v in pairs]


def make_status(typestore, level: int, name: str, message: str, pairs: list[tuple[str, Any]]):
    cls = typestore.types["diagnostic_msgs/msg/DiagnosticStatus"]
    return cls(
        level=int(level),
        name=name,
        message=message,
        hardware_id="a3",
        values=key_values(typestore, pairs),
    )


def write_ros2(writer: Writer, channel_id: int, timestamp_ns: int, typestore, typename: str, msg: Any):
    data = bytes(typestore.serialize_cdr(msg, typename))
    writer.add_message(channel_id, log_time=timestamp_ns, publish_time=timestamp_ns, data=data)


def group_joint_indices(group: str) -> list[tuple[int, str]]:
    cfg = GROUPS[group]
    start = int(cfg["start"])
    return [(start + offset, name) for offset, name in enumerate(cfg["names"])]


def plot_panel_id(group: str, quantity: str, joint_index: int) -> str:
    return f"Plot!{group}-{quantity}-{joint_index}"


def joint_message_path(topic: str, field: str, joint_index: int) -> str:
    return f"{topic}.{field}[{joint_index}]"


def stack_layout(panel_ids: list[str], direction: str = "column") -> Any:
    if len(panel_ids) == 1:
        return panel_ids[0]
    return {
        "first": panel_ids[0],
        "second": stack_layout(panel_ids[1:], direction),
        "direction": direction,
        "splitPercentage": round(100.0 / len(panel_ids), 3),
    }


def joint_plot_grid_layout(panel_ids: list[str]) -> Any:
    mid = (len(panel_ids) + 1) // 2
    return {
        "first": stack_layout(panel_ids[:mid]),
        "second": stack_layout(panel_ids[mid:]),
        "direction": "row",
        "splitPercentage": 50,
    }


def plot_series(path: str, label: str, color: str) -> dict[str, Any]:
    return {
        "value": path,
        "label": label,
        "color": color,
        "enabled": True,
        "timestampMethod": "receiveTime",
        "showLine": True,
        "lineSize": 1.5,
    }


def joint_plot_config(
    title: str,
    field: str,
    y_axis_label: str,
    joint_index: int,
    actual_label: str,
    desired_label: str,
) -> dict[str, Any]:
    return {
        PANEL_TITLE_CONFIG_KEY: title,
        "title": title,
        "paths": [
            plot_series(
                joint_message_path("/a3_debug/actual_joint_states_31", field, joint_index),
                actual_label,
                ACTUAL_SERIES_COLOR,
            ),
            plot_series(
                joint_message_path("/a3_debug/desired_joint_states_31", field, joint_index),
                desired_label,
                COMMAND_SERIES_COLOR,
            ),
        ],
        "xAxisVal": "timestamp",
        "yAxisLabel": y_axis_label,
        "showLegend": False,
        "legendDisplay": "none",
        "showPlotValuesInLegend": False,
        "showXAxisLabels": True,
        "showYAxisLabels": True,
        "isSynced": True,
        "sidebarDimension": 0,
    }


def comparison_tab_markdown(
    tab_title: str,
    group: str,
    field: str,
    actual_label: str,
    desired_label: str,
) -> str:
    lines = [
        f"### {tab_title}",
        "",
        "- Add one Plot panel per joint in this tab.",
        "- X-axis: Timestamp. Enable plot sync.",
        "- Hide the legend, or set it to Top if you temporarily need labels.",
        "- Each Plot panel has two series:",
    ]
    for index, name in group_joint_indices(group):
        lines.append(
            f"  - `{name}` [{index}]: {actual_label} "
            f"`{joint_message_path('/a3_debug/actual_joint_states_31', field, index)}`; "
            f"{desired_label} "
            f"`{joint_message_path('/a3_debug/desired_joint_states_31', field, index)}`"
        )
    return "\n".join(lines)


def write_panel_settings(
    output_dir: Path,
    urdf_path: Path,
    mesh_url_base: str | None = None,
    layout_urdf_path: Path | None = None,
):
    comparison_tabs = "\n\n".join(
        comparison_tab_markdown(tab_title, group, field, actual_label, desired_label)
        for tab_title, group, _quantity, field, actual_label, desired_label, _y_label in PLOT_TAB_SPECS
    )
    settings = f"""# Foxglove panel settings for A3 body-drive debug

Open `a3_body_drive_debug.mcap` in Foxglove and import
`a3_body_drive_debug.layout.json`. If the layout JSON is not compatible with
your Foxglove version, configure panels manually with these settings. The same
stable layout template is also shipped in the package at
`config/a3_body_drive_debug.layout.json`.

## Layout

- Left side: one 3D panel for the robot.
- Right side: one Tab panel.
- Create these tabs: `waist pos`, `waist tau`, `waist vel`,
  `leg pos`, `leg tau`, `leg vel`, `arm pos`, `arm tau`, `arm vel`.
- In every tab, add one Plot panel per joint. Put state and command for the
  same joint in the same Plot panel.
- Plot legends are hidden by default to keep dense subplot tabs readable.
  Blue is state/measured, orange is command/PD command.

## 3D panel

- Add a URDF custom layer.
- Source: Topic.
- Topic: `/robot_description`.
- Meshes are stored as MCAP attachments named by the URDF `package://a3/...`
  URLs, so Foxglove Desktop can open the MCAP without a separate asset server
  or local `file://` mesh fetches.
- Display frame / fixed frame: `world`.
- The converter repeats `/robot_description` periodically so Foxglove can load
  the URDF even if the 3D panel starts after the first MCAP message.
- If you use browser Foxglove and the robot still does not appear, use
  Foxglove Desktop or re-run conversion with `--mesh-url-base` and serve the
  output assets over HTTP.
- HTTP mesh URL base: `{mesh_url_base or "not set; using MCAP mesh attachments"}`.
- `/tf` is derived from `/body_drive/pelvis_imu/data` as `world -> pelvis_link`
  with zero translation. This keeps the world/grid static and shows pelvis
  attitude, but it does not estimate global XY motion.
- Control mode: Joint states.
- Joint states: `/joint_states`.
- If the robot is visible but static, make sure `/joint_states` is visible in
  the 3D panel topic list so the panel subscribes to joint updates.
- `/joint_states` contains every joint from the rewritten URDF; non-body-drive
  fixed joints are filled with zeros for Foxglove compatibility.
- Display mode: Visual.
- Fixed frame / display frame: `pelvis_link` if available, otherwise leave the default.
- The rewritten URDF is also stored at: `{urdf_path}`.

## Right-side comparison tabs

{comparison_tabs}

## Per-joint table

- Panel type: Table or Raw Messages.
- Topic: `/a3_debug/joint_error_table`.
- Each DiagnosticStatus entry is one joint, with actual, desired, and error
  values in its key/value list.

## Timing / raw diagnostics

- Panel type: Raw Messages.
- Topics:
  - `/a3_debug/timing`
  - `/tf`
  - `/body_drive/waist_joint_state`
  - `/body_drive/leg_joint_state`
  - `/body_drive/arm_joint_state`
  - `/body_drive/neck_joint_state`
  - `/body_drive/waist_joint_command`
  - `/body_drive/leg_joint_command`
  - `/body_drive/arm_joint_command`
  - `/body_drive/neck_joint_command`
  - `/body_drive/pelvis_imu/data`
  - `/body_drive/torso_imu/data`
"""
    (output_dir / "foxglove_panel_settings.md").write_text(settings, encoding="utf-8")


def write_layout(output_dir: Path, urdf_file_path: Path | None = None):
    if urdf_file_path is not None:
        urdf_layer = {
            "visible": True,
            "instanceId": "a3_urdf",
            "layerId": "foxglove.Urdf",
            "sourceType": "filePath",
            "framePrefix": "",
            "type": "urdf",
            "label": "A3 actual",
            "source": "filePath",
            "filePath": str(urdf_file_path.resolve()),
            "controlMode": "jointStates",
            "jointStatesTopic": "/joint_states",
            "displayMode": "visual",
            "showOutlines": False,
        }
    else:
        urdf_layer = {
            "visible": True,
            "instanceId": "a3_urdf",
            "layerId": "foxglove.Urdf",
            "sourceType": "topic",
            "framePrefix": "",
            "type": "urdf",
            "label": "A3 actual",
            "source": "topic",
            "topic": "/robot_description",
            "controlMode": "jointStates",
            "jointStatesTopic": "/joint_states",
            "displayMode": "visual",
            "showOutlines": False,
        }

    config_by_id: dict[str, Any] = {
        "3D!a3": {
            "cameraState": {
                "distance": 3.0,
                "perspective": True,
                "phi": 1.0,
                "target": [0, 0, 0.8],
                "thetaOffset": 0.5,
            },
            "followMode": "follow-none",
            "followTf": "world",
            "fixedFrame": "world",
            "scene": {
                "backgroundColor": "#111318",
                "labelScaleFactor": 1.0,
                "transforms": {"enablePreloading": True},
            },
            "topics": {
                "/robot_description": {"visible": urdf_file_path is None},
                "/tf": {"visible": True},
                "/joint_states": {"visible": True},
            },
            "layers": {
                "grid": {
                    "visible": True,
                    "instanceId": "grid",
                    "layerId": "foxglove.Grid",
                    "label": "Grid",
                    "frameId": "world",
                    "size": 4,
                    "divisions": 20,
                    "lineWidth": 1,
                    "color": "#248eff",
                    "position": [0, 0, 0],
                    "rotation": [0, 0, 0],
                },
                "a3_urdf": urdf_layer,
            },
        }
    }

    tabs = []
    for tab_title, group, quantity, field, actual_label, desired_label, y_axis_label in PLOT_TAB_SPECS:
        panel_ids = []
        for joint_index, joint_name in group_joint_indices(group):
            panel_id = plot_panel_id(group, quantity, joint_index)
            panel_ids.append(panel_id)
            config_by_id[panel_id] = joint_plot_config(
                f"{joint_name} {quantity}",
                field,
                y_axis_label,
                joint_index,
                actual_label,
                desired_label,
            )
        tabs.append({"title": tab_title, "layout": joint_plot_grid_layout(panel_ids)})

    config_by_id["Tab!a3-body-drive-debug"] = {
        "activeTabIdx": 0,
        "tabs": tabs,
    }

    layout = {
        "configById": config_by_id,
        "globalVariables": {},
        "userNodes": {},
        "playbackConfig": {"speed": 1},
        "savedProps": config_by_id,
        "layout": {
            "first": "3D!a3",
            "second": "Tab!a3-body-drive-debug",
            "direction": "row",
            "splitPercentage": 40,
        },
    }
    (output_dir / "a3_body_drive_debug.layout.json").write_text(
        json.dumps(layout, indent=2, sort_keys=True), encoding="utf-8"
    )


def parse_compression(name: str) -> CompressionType:
    value = name.strip().lower().replace("-", "_")
    if value == "none":
        return CompressionType.NONE
    if value == "lz4":
        return CompressionType.LZ4
    if value == "zstd":
        return CompressionType.ZSTD
    raise ValueError(f"unsupported compression: {name}")


def fmt_size(num_bytes: float) -> str:
    units = ["B", "KiB", "MiB", "GiB", "TiB"]
    value = float(num_bytes)
    for unit in units:
        if abs(value) < 1024.0 or unit == units[-1]:
            return f"{value:.1f}{unit}"
        value /= 1024.0
    return f"{value:.1f}TiB"


def convert(args: argparse.Namespace) -> int:
    raw_path = Path(args.raw_path).resolve()
    mcap_files = find_mcap_files(raw_path)
    input_bytes_total = sum(path.stat().st_size for path in mcap_files)
    session_root = infer_session_root(raw_path)
    output_dir = Path(args.output).resolve() if args.output else session_root / "foxglove"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_assets_dir = output_dir / "assets"
    output_assets_dir.mkdir(parents=True, exist_ok=True)

    typestore = get_ros2_typestore()
    asset_dir = find_a3_asset_dir(Path(__file__).resolve(), args.asset_dir)
    mesh_url_base = getattr(args, "mesh_url_base", None)
    mesh_mode = getattr(args, "mesh_mode", "attachment")
    if mesh_url_base:
        mesh_mode = "url"
    robot_description, mesh_attachments = copy_and_rewrite_urdf(
        asset_dir,
        output_assets_dir,
        mesh_url_base=mesh_url_base,
        mesh_mode=mesh_mode,
    )
    urdf_joint_names = extract_urdf_joint_names(robot_description)
    local_urdf_path = output_assets_dir / "a3" / "model.urdf"
    rewritten_urdf_path = output_assets_dir / "a3" / "model.foxglove.urdf"
    layout_urdf_path = None

    out_mcap = output_dir / "a3_body_drive_debug.mcap"
    if out_mcap.exists():
        out_mcap.unlink()

    copy_raw = bool(getattr(args, "copy_raw", True))
    compression_name = getattr(args, "compression", "zstd")
    compression = parse_compression(compression_name)
    progress_interval_s = float(getattr(args, "progress_interval", 5.0))
    if progress_interval_s < 0.0:
        progress_interval_s = 0.0

    actual_q = np.zeros(31, dtype=np.float64)
    actual_dq = np.zeros(31, dtype=np.float64)
    actual_tau = np.zeros(31, dtype=np.float64)
    desired_q = np.zeros(31, dtype=np.float64)
    desired_dq = np.zeros(31, dtype=np.float64)
    tau_ff = np.zeros(31, dtype=np.float64)
    kp = np.zeros(31, dtype=np.float64)
    kd = np.zeros(31, dtype=np.float64)
    state_seen: dict[str, int] = {}
    command_seen: dict[str, int] = {}
    latest_state_raw: dict[str, tuple[bytes, int]] = {}
    latest_command_raw: dict[str, tuple[bytes, int]] = {}
    latest_pelvis_imu_raw: tuple[bytes, int] | None = None
    decoded_state_log_time: dict[str, int] = {}
    decoded_command_log_time: dict[str, int] = {}
    decoded_pelvis_imu_log_time = -1
    decoded_pelvis_imu: Any | None = None
    decoded_pelvis_imu_stamp_ns = 0

    derived_interval_ns = 0 if args.derived_hz <= 0 else int(1_000_000_000 / args.derived_hz)
    next_derived_ns = 0
    raw_schema_map: dict[tuple[str, str, bytes], int] = {}
    raw_channel_map: dict[int, int] = {}
    robot_description_written = False
    last_robot_description_ns = -1
    robot_description_count = 0
    raw_count = 0
    raw_copied_count = 0
    derived_count = 0
    decoded_count = 0

    start_mono = time.monotonic()
    last_progress_mono = start_mono
    completed_input_bytes = 0

    def log_progress(
        current_file: Path | None = None,
        current_file_offset: int = 0,
        force: bool = False,
    ):
        nonlocal last_progress_mono
        now = time.monotonic()
        if not force and (progress_interval_s <= 0.0 or now - last_progress_mono < progress_interval_s):
            return
        last_progress_mono = now

        processed_input_bytes = completed_input_bytes
        if current_file is not None:
            processed_input_bytes += max(0, current_file_offset)
        elapsed = max(now - start_mono, 1e-6)
        msg_rate = raw_count / elapsed
        out_size = out_mcap.stat().st_size if out_mcap.exists() else 0
        pct = 100.0 * processed_input_bytes / input_bytes_total if input_bytes_total > 0 else 0.0
        file_label = current_file.name if current_file is not None else "-"
        print(
            "[a3-debug-convert] progress "
            f"input={pct:5.1f}% file={file_label} "
            f"read={raw_count} copied={raw_copied_count} decoded={decoded_count} "
            f"derived={derived_count} out={fmt_size(out_size)} "
            f"rate={msg_rate:.0f} msg/s",
            flush=True,
        )

    robot_description_period = getattr(args, "robot_description_period", None)
    if robot_description_period is None:
        robot_description_period = 0.0 if mesh_mode == "data-uri" else 5.0
    robot_description_period_ns = int(max(0.0, float(robot_description_period)) * 1_000_000_000)

    with out_mcap.open("wb") as stream:
        writer = Writer(stream, compression=compression)
        writer.start()
        channels = register_derived_channels(writer, typestore)

        def ensure_raw_channel(schema, channel) -> int:
            if channel.id in raw_channel_map:
                return raw_channel_map[channel.id]
            schema_data = bytes(schema.data or b"")
            schema_key = (schema.name, schema.encoding, schema_data)
            if schema_key not in raw_schema_map:
                raw_schema_map[schema_key] = writer.register_schema(
                    schema.name, schema.encoding, schema_data
                )
            out_channel = writer.register_channel(
                channel.topic,
                channel.message_encoding,
                raw_schema_map[schema_key],
                dict(channel.metadata or {}),
            )
            raw_channel_map[channel.id] = out_channel
            return out_channel

        def all_state_seen() -> bool:
            return set(state_seen) == set(GROUPS)

        def all_command_seen() -> bool:
            return set(command_seen) == set(GROUPS)

        def store_latest_state(group: str, data: Any, log_time_ns: int):
            latest_state_raw[group] = (
                data if isinstance(data, bytes) else bytes(data),
                log_time_ns,
            )

        def store_latest_command(group: str, data: Any, log_time_ns: int):
            latest_command_raw[group] = (
                data if isinstance(data, bytes) else bytes(data),
                log_time_ns,
            )

        def store_latest_pelvis_imu(data: Any, log_time_ns: int):
            nonlocal latest_pelvis_imu_raw
            latest_pelvis_imu_raw = (
                data if isinstance(data, bytes) else bytes(data),
                log_time_ns,
            )

        def refresh_latest_inputs():
            nonlocal decoded_count
            nonlocal decoded_pelvis_imu_log_time
            nonlocal decoded_pelvis_imu
            nonlocal decoded_pelvis_imu_stamp_ns

            for group, (data, log_time_ns) in latest_state_raw.items():
                if decoded_state_log_time.get(group) == log_time_ns:
                    continue
                decoded = typestore.deserialize_cdr(data, "joint_msgs/msg/JointState")
                decoded_count += 1
                cfg = GROUPS[group]
                pos, vel, eff = values_by_name(decoded.joints, cfg["names"], ["position", "velocity", "effort"])
                start = int(cfg["start"])
                stop = start + len(cfg["names"])
                actual_q[start:stop] = pos
                actual_dq[start:stop] = vel
                actual_tau[start:stop] = eff
                state_seen[group] = msg_stamp_ns(decoded, log_time_ns)
                decoded_state_log_time[group] = log_time_ns

            for group, (data, log_time_ns) in latest_command_raw.items():
                if decoded_command_log_time.get(group) == log_time_ns:
                    continue
                decoded = typestore.deserialize_cdr(data, "joint_msgs/msg/JointCommand")
                decoded_count += 1
                cfg = GROUPS[group]
                pos, vel, eff, stiff, damp = values_by_name(
                    decoded.joints,
                    cfg["names"],
                    ["position", "velocity", "effort", "stiffness", "damping"],
                )
                start = int(cfg["start"])
                stop = start + len(cfg["names"])
                desired_q[start:stop] = pos
                desired_dq[start:stop] = vel
                tau_ff[start:stop] = eff
                kp[start:stop] = stiff
                kd[start:stop] = damp
                command_seen[group] = msg_stamp_ns(decoded, log_time_ns)
                decoded_command_log_time[group] = log_time_ns

            if latest_pelvis_imu_raw is not None:
                data, log_time_ns = latest_pelvis_imu_raw
                if decoded_pelvis_imu_log_time != log_time_ns:
                    decoded = typestore.deserialize_cdr(data, "sensor_msgs/msg/Imu")
                    decoded_count += 1
                    decoded_pelvis_imu = decoded
                    decoded_pelvis_imu_stamp_ns = msg_stamp_ns(decoded, log_time_ns)
                    decoded_pelvis_imu_log_time = log_time_ns

        def emit_derived(timestamp_ns: int):
            nonlocal next_derived_ns
            nonlocal robot_description_written
            nonlocal last_robot_description_ns
            nonlocal robot_description_count
            nonlocal derived_count
            if derived_interval_ns > 0 and timestamp_ns < next_derived_ns:
                return
            next_derived_ns = timestamp_ns + derived_interval_ns
            refresh_latest_inputs()

            should_write_robot_description = (
                not robot_description_written
                or (
                    robot_description_period_ns > 0
                    and timestamp_ns - last_robot_description_ns >= robot_description_period_ns
                )
            )
            if should_write_robot_description:
                string_cls = typestore.types["std_msgs/msg/String"]
                write_ros2(
                    writer,
                    channels.robot_description,
                    timestamp_ns,
                    typestore,
                    "std_msgs/msg/String",
                    string_cls(data=robot_description),
                )
                robot_description_written = True
                last_robot_description_ns = timestamp_ns
                robot_description_count += 1

            if decoded_pelvis_imu is not None:
                write_ros2(
                    writer,
                    channels.tf,
                    decoded_pelvis_imu_stamp_ns,
                    typestore,
                    "tf2_msgs/msg/TFMessage",
                    make_pelvis_tf_from_imu(typestore, decoded_pelvis_imu_stamp_ns, decoded_pelvis_imu),
                )

            statuses = []
            missing_state = sorted(set(GROUPS) - set(state_seen))
            missing_command = sorted(set(GROUPS) - set(command_seen))
            statuses.append(
                make_status(
                    typestore,
                    0 if not missing_state else 1,
                    "state_groups",
                    "complete" if not missing_state else "missing",
                    [("missing", ",".join(missing_state) or "none")],
                )
            )
            statuses.append(
                make_status(
                    typestore,
                    0 if not missing_command else 1,
                    "command_groups",
                    "complete" if not missing_command else "missing",
                    [("missing", ",".join(missing_command) or "none")],
                )
            )
            for group in GROUPS:
                if group in state_seen:
                    statuses.append(
                        make_status(
                            typestore,
                            0,
                            f"state_age/{group}",
                            "ok",
                            [("age_ms", f"{(timestamp_ns - state_seen[group]) / 1e6:.3f}")],
                        )
                    )
                if group in command_seen:
                    statuses.append(
                        make_status(
                            typestore,
                            0,
                            f"command_age/{group}",
                            "ok",
                            [("age_ms", f"{(timestamp_ns - command_seen[group]) / 1e6:.3f}")],
                        )
                    )
            write_ros2(
                writer,
                channels.timing,
                timestamp_ns,
                typestore,
                "diagnostic_msgs/msg/DiagnosticArray",
                make_diag(typestore, timestamp_ns, statuses),
            )

            if not all_state_seen():
                return

            urdf_q, urdf_dq, urdf_tau = values_for_joint_names(
                urdf_joint_names, actual_q, actual_dq, actual_tau
            )
            write_ros2(
                writer,
                channels.joint_states,
                timestamp_ns,
                typestore,
                "sensor_msgs/msg/JointState",
                make_joint_state(
                    typestore,
                    timestamp_ns,
                    urdf_joint_names,
                    urdf_q,
                    urdf_dq,
                    urdf_tau,
                ),
            )
            write_ros2(
                writer,
                channels.actual,
                timestamp_ns,
                typestore,
                "sensor_msgs/msg/JointState",
                make_joint_state(typestore, timestamp_ns, A3_JOINT_NAMES_31, actual_q, actual_dq, actual_tau),
            )

            if not all_command_seen():
                derived_count += 1
                return

            tau_des_pd = tau_ff + kp * (desired_q - actual_q) + kd * (desired_dq - actual_dq)
            q_err = desired_q - actual_q
            dq_err = desired_dq - actual_dq
            tau_err = tau_des_pd - actual_tau
            write_ros2(
                writer,
                channels.desired,
                timestamp_ns,
                typestore,
                "sensor_msgs/msg/JointState",
                make_joint_state(typestore, timestamp_ns, A3_JOINT_NAMES_31, desired_q, desired_dq, tau_des_pd),
            )
            write_ros2(
                writer,
                channels.error,
                timestamp_ns,
                typestore,
                "sensor_msgs/msg/JointState",
                make_joint_state(typestore, timestamp_ns, A3_JOINT_NAMES_31, q_err, dq_err, tau_err),
            )
            table_statuses = []
            for i, name in enumerate(A3_JOINT_NAMES_31):
                level = 1 if abs(q_err[i]) >= args.warn_pos_error else 0
                table_statuses.append(
                    make_status(
                        typestore,
                        level,
                        name,
                        "position_error_warn" if level else "ok",
                        [
                            ("actual_position", f"{actual_q[i]:.9g}"),
                            ("desired_position", f"{desired_q[i]:.9g}"),
                            ("position_error", f"{q_err[i]:.9g}"),
                            ("actual_velocity", f"{actual_dq[i]:.9g}"),
                            ("desired_velocity", f"{desired_dq[i]:.9g}"),
                            ("velocity_error", f"{dq_err[i]:.9g}"),
                            ("measured_torque", f"{actual_tau[i]:.9g}"),
                            ("tau_ff", f"{tau_ff[i]:.9g}"),
                            ("pd_desired_torque", f"{tau_des_pd[i]:.9g}"),
                            ("torque_error", f"{tau_err[i]:.9g}"),
                            ("kp", f"{kp[i]:.9g}"),
                            ("kd", f"{kd[i]:.9g}"),
                        ],
                    )
                )
            write_ros2(
                writer,
                channels.table,
                timestamp_ns,
                typestore,
                "diagnostic_msgs/msg/DiagnosticArray",
                make_diag(typestore, timestamp_ns, table_statuses),
            )
            derived_count += 1

        print(
            "[a3-debug-convert] start "
            f"files={len(mcap_files)} input={fmt_size(input_bytes_total)} "
            f"copy_raw={int(copy_raw)} compression={compression_name} "
            f"derived_hz={args.derived_hz} mesh_mode={mesh_mode} "
            f"robot_description_period={robot_description_period}",
            flush=True,
        )

        for file_index, mcap_file in enumerate(mcap_files, start=1):
            file_size = mcap_file.stat().st_size
            print(
                f"[a3-debug-convert] file {file_index}/{len(mcap_files)} "
                f"{mcap_file.name} size={fmt_size(file_size)}",
                flush=True,
            )
            with mcap_file.open("rb") as stream_in:
                reader = make_reader(stream_in)
                for schema, channel, message in reader.iter_messages():
                    raw_count += 1
                    data = message.data
                    if copy_raw:
                        out_channel = ensure_raw_channel(schema, channel)
                        writer.add_message(
                            out_channel,
                            log_time=int(message.log_time),
                            publish_time=int(message.publish_time or message.log_time),
                            data=data,
                            sequence=int(message.sequence or 0),
                        )
                        raw_copied_count += 1

                    msg_type = schema.name
                    try:
                        if channel.topic in STATE_TOPIC_TO_GROUP and msg_type == "joint_msgs/msg/JointState":
                            group = STATE_TOPIC_TO_GROUP[channel.topic]
                            store_latest_state(group, data, int(message.log_time))
                            emit_derived(int(message.log_time))
                        elif channel.topic in COMMAND_TOPIC_TO_GROUP and msg_type == "joint_msgs/msg/JointCommand":
                            group = COMMAND_TOPIC_TO_GROUP[channel.topic]
                            store_latest_command(group, data, int(message.log_time))
                            emit_derived(int(message.log_time))
                        elif channel.topic == "/body_drive/pelvis_imu/data" and msg_type == "sensor_msgs/msg/Imu":
                            store_latest_pelvis_imu(data, int(message.log_time))
                            emit_derived(int(message.log_time))
                    except Exception as exc:
                        print(
                            f"[a3-debug-convert] warning: failed to decode {channel.topic} "
                            f"at {message.log_time}: {exc}",
                            file=sys.stderr,
                        )
                    if progress_interval_s > 0.0:
                        now = time.monotonic()
                        if now - last_progress_mono >= progress_interval_s:
                            log_progress(mcap_file, stream_in.tell(), force=True)

            completed_input_bytes += file_size
            log_progress(None, force=True)

        if not robot_description_written:
            string_cls = typestore.types["std_msgs/msg/String"]
            write_ros2(
                writer,
                channels.robot_description,
                0,
                typestore,
                "std_msgs/msg/String",
                string_cls(data=robot_description),
            )
            robot_description_written = True
            robot_description_count += 1
        mesh_attachment_bytes = 0
        for attachment in mesh_attachments:
            data = attachment.path.read_bytes()
            mesh_attachment_bytes += len(data)
            writer.add_attachment(
                create_time=0,
                log_time=0,
                name=attachment.name,
                media_type=attachment.media_type,
                data=data,
            )
        writer.finish()

    write_layout(output_dir, urdf_file_path=layout_urdf_path)
    write_panel_settings(
        output_dir,
        rewritten_urdf_path,
        mesh_url_base=mesh_url_base,
        layout_urdf_path=layout_urdf_path,
    )
    print(f"[a3-debug-convert] raw messages read: {raw_count}")
    print(f"[a3-debug-convert] raw messages copied: {raw_copied_count}")
    print(f"[a3-debug-convert] decoded messages: {decoded_count}")
    print(f"[a3-debug-convert] derived frames: {derived_count}")
    print(f"[a3-debug-convert] robot_description messages: {robot_description_count}")
    print(
        f"[a3-debug-convert] mesh attachments: {len(mesh_attachments)} "
        f"({fmt_size(mesh_attachment_bytes)})"
    )
    print(f"[a3-debug-convert] output: {out_mcap}")
    print(f"[a3-debug-convert] layout: {output_dir / 'a3_body_drive_debug.layout.json'}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_path", help="Raw AimRT bag directory, raw parent directory, or .mcap file")
    parser.add_argument("--output", help="Output foxglove directory; default is <session>/foxglove")
    parser.add_argument(
        "--asset-dir",
        help=(
            "Optional A3 URDF asset directory containing model.urdf and "
            "meshes/. Not bundled with the deploy package."
        ),
    )
    parser.add_argument(
        "--mesh-url-base",
        help=(
            "Rewrite URDF mesh filenames to this HTTP(S) base URL, for browser Foxglove. "
            "Example: http://127.0.0.1:8765/assets/a3"
        ),
    )
    parser.add_argument(
        "--mesh-mode",
        choices=("attachment", "data-uri"),
        default="attachment",
        help=(
            "How to store local URDF meshes when --mesh-url-base is not set. "
            "attachment uses MCAP attachments; data-uri embeds visual meshes in /robot_description. "
            "Default: attachment"
        ),
    )
    parser.add_argument(
        "--derived-hz",
        type=float,
        default=100.0,
        help="Maximum derived debug topic rate. Use 0 for every raw update. Default: 100",
    )
    parser.add_argument(
        "--warn-pos-error",
        type=float,
        default=0.2,
        help="Position error threshold for WARN entries in joint_error_table. Default: 0.2 rad",
    )
    parser.add_argument(
        "--no-raw",
        dest="copy_raw",
        action="store_false",
        help="Do not copy original raw topics into the output MCAP. Much faster/smaller; derived Foxglove topics are still generated.",
    )
    parser.add_argument(
        "--compression",
        choices=("zstd", "lz4", "none"),
        default="zstd",
        help="Output MCAP chunk compression. Use 'none' for fastest conversion. Default: zstd",
    )
    parser.add_argument(
        "--progress-interval",
        type=float,
        default=5.0,
        help="Seconds between progress logs. Use 0 to disable. Default: 5",
    )
    parser.add_argument(
        "--robot-description-period",
        type=float,
        default=None,
        help=(
            "Seconds between repeated /robot_description messages. Use 0 to write only once. "
            "Default: 5 for attachment/url meshes, 0 for data-uri meshes."
        ),
    )
    args = parser.parse_args()
    return convert(args)


if __name__ == "__main__":
    raise SystemExit(main())
