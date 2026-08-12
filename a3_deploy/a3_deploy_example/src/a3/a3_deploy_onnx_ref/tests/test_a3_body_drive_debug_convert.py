from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

mcap_reader = pytest.importorskip("mcap.reader")
mcap_writer = pytest.importorskip("mcap.writer")
yaml = pytest.importorskip("yaml")


REPO_ROOT = Path(__file__).resolve().parents[5]
DEBUG_ROOT = REPO_ROOT / "a3_deploy_example/src/a3/a3_deploy_onnx_ref"
CONVERTER = DEBUG_ROOT / "scripts/tools/a3_body_drive_debug_convert.py"
STATIC_LAYOUT = DEBUG_ROOT / "config/a3_body_drive_debug.layout.json"


def load_converter():
    spec = importlib.util.spec_from_file_location("a3_body_drive_debug_convert", CONVERTER)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_test_asset_dir(tmp_path: Path, conv) -> Path:
    asset_dir = tmp_path / "asset_src"
    (asset_dir / "meshes").mkdir(parents=True)
    (asset_dir / "meshes/pelvis_link.STL").write_bytes(b"fake-stl")
    joints = "\n".join(
        f'<joint name="{name}" type="fixed">'
        '<parent link="pelvis_link"/>'
        f'<child link="{name}_link"/>'
        "</joint>"
        for name in [*conv.A3_JOINT_NAMES_31, "debug_extra_joint"]
    )
    (asset_dir / "model.urdf").write_text(
        '<robot name="a3"><link name="pelvis_link"><visual><geometry>'
        '<mesh filename="meshes/pelvis_link.STL"/>'
        f"</geometry></visual></link>{joints}</robot>",
        encoding="utf-8",
    )
    return asset_dir


def test_record_defaults_and_rotation_config():
    record_script = (DEBUG_ROOT / "scripts/run_a3_body_drive_debug_record.sh").read_text()
    assert 'TRANSPORT="iceoryx"' in record_script
    assert "--ros2" in record_script

    expected_topics = {
        "/body_drive/waist_joint_state",
        "/body_drive/leg_joint_state",
        "/body_drive/arm_joint_state",
        "/body_drive/neck_joint_state",
        "/body_drive/waist_joint_command",
        "/body_drive/leg_joint_command",
        "/body_drive/arm_joint_command",
        "/body_drive/neck_joint_command",
        "/body_drive/pelvis_imu/data",
        "/body_drive/torso_imu/data",
    }

    for transport in ("iceoryx", "ros2"):
        cfg_path = DEBUG_ROOT / f"config/a3_body_drive_debug_record.{transport}.yaml"
        cfg = yaml.safe_load(cfg_path.read_text())
        action = cfg["aimrt"]["plugin"]["plugins"][-1]["options"]["record_actions"][0]["options"]
        storage = action["storage_policy"]
        topics = action["topic_meta_list"]
        assert storage["max_bag_size_m"] == 256
        assert storage["max_bag_num"] == 1
        assert {topic["topic_name"] for topic in topics} == expected_topics
        assert all(topic["serialization_type"] == "ros2" for topic in topics)
        assert all(topic["record_enabled"] for topic in topics)


def test_a3_body_drive_group_order_is_31_dof_order():
    conv = load_converter()
    flattened = []
    for group in ("waist", "neck", "arm", "leg"):
        cfg = conv.GROUPS[group]
        assert conv.A3_JOINT_NAMES_31[int(cfg["start"]): int(cfg["start"]) + len(cfg["names"])] == cfg["names"]
        flattened.extend(cfg["names"])
    assert flattened == conv.A3_JOINT_NAMES_31
    assert len(flattened) == 31


def test_urdf_mesh_url_rewrite_for_browser_foxglove(tmp_path):
    conv = load_converter()
    asset_dir = write_test_asset_dir(tmp_path, conv)

    robot_description, attachments = conv.copy_and_rewrite_urdf(
        asset_dir,
        tmp_path / "out_assets_http",
        mesh_url_base="http://127.0.0.1:8765/assets/a3/",
    )

    assert 'filename="http://127.0.0.1:8765/assets/a3/meshes/pelvis_link.STL"' in robot_description
    assert attachments == []
    assert (tmp_path / "out_assets_http/a3/meshes/pelvis_link.STL").is_file()
    assert (tmp_path / "out_assets_http/a3/model.foxglove.urdf").is_file()

    robot_description, attachments = conv.copy_and_rewrite_urdf(asset_dir, tmp_path / "out_assets_pkg")
    assert 'filename="package://a3/meshes/pelvis_link.STL"' in robot_description
    assert len(attachments) == 1
    assert attachments[0].name == "package://a3/meshes/pelvis_link.STL"
    assert attachments[0].media_type == "model/stl"
    assert attachments[0].path == (tmp_path / "out_assets_pkg/a3/meshes/pelvis_link.STL").resolve()

    robot_description, attachments = conv.copy_and_rewrite_urdf(
        asset_dir,
        tmp_path / "out_assets_data_uri",
        mesh_mode="data-uri",
    )
    assert 'filename="data:model/stl;base64,' in robot_description
    assert "#pelvis_link.STL" in robot_description
    assert "package://a3/meshes/pelvis_link.STL" not in robot_description
    assert attachments == []


def test_converter_generates_pd_torque_and_error_topics(tmp_path):
    conv = load_converter()
    typestore = conv.get_ros2_typestore()
    header_cls = typestore.types["std_msgs/msg/Header"]
    time_cls = typestore.types["builtin_interfaces/msg/Time"]
    state_cls = typestore.types["joint_msgs/msg/State"]
    command_cls = typestore.types["joint_msgs/msg/Command"]
    joint_state_cls = typestore.types["joint_msgs/msg/JointState"]
    joint_command_cls = typestore.types["joint_msgs/msg/JointCommand"]
    imu_cls = typestore.types["sensor_msgs/msg/Imu"]
    quaternion_cls = typestore.types["geometry_msgs/msg/Quaternion"]
    vector_cls = typestore.types["geometry_msgs/msg/Vector3"]

    raw = tmp_path / "raw"
    raw.mkdir()
    raw_mcap = raw / "aimrtbag_smoke.mcap"

    def header(timestamp_ns: int):
        return header_cls(
            stamp=time_cls(sec=timestamp_ns // 1_000_000_000, nanosec=timestamp_ns % 1_000_000_000),
            frame_id="pelvis_link",
        )

    def schema_data(typename: str) -> bytes:
        msgdef, _ = typestore.generate_msgdef(typename, ros_version=2)
        return msgdef.encode()

    with raw_mcap.open("wb") as stream:
        writer = mcap_writer.Writer(stream, compression=mcap_writer.CompressionType.NONE)
        writer.start()
        state_schema = writer.register_schema(
            "joint_msgs/msg/JointState", "ros2msg", schema_data("joint_msgs/msg/JointState")
        )
        command_schema = writer.register_schema(
            "joint_msgs/msg/JointCommand", "ros2msg", schema_data("joint_msgs/msg/JointCommand")
        )
        imu_schema = writer.register_schema(
            "sensor_msgs/msg/Imu", "ros2msg", schema_data("sensor_msgs/msg/Imu")
        )
        state_channels = {}
        command_channels = {}
        for group, cfg in conv.GROUPS.items():
            state_channels[group] = writer.register_channel(cfg["state_topic"], "cdr", state_schema)
            command_channels[group] = writer.register_channel(cfg["command_topic"], "cdr", command_schema)
        imu_channel = writer.register_channel("/body_drive/pelvis_imu/data", "cdr", imu_schema)

        timestamp_ns = 1_700_000_000_000_000_000
        imu_msg = imu_cls(
            header=header(timestamp_ns),
            orientation=quaternion_cls(x=0.0, y=0.0, z=0.1, w=0.995),
            orientation_covariance=np.zeros(9, dtype=np.float64),
            angular_velocity=vector_cls(x=0.0, y=0.0, z=0.0),
            angular_velocity_covariance=np.zeros(9, dtype=np.float64),
            linear_acceleration=vector_cls(x=0.0, y=0.0, z=9.8),
            linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
        )
        writer.add_message(
            imu_channel,
            timestamp_ns,
            bytes(typestore.serialize_cdr(imu_msg, "sensor_msgs/msg/Imu")),
            timestamp_ns,
        )
        for group, cfg in conv.GROUPS.items():
            start = int(cfg["start"])
            states = []
            commands = []
            for offset, name in enumerate(cfg["names"]):
                index = start + offset
                states.append(
                    state_cls(
                        name=name,
                        sequence=index,
                        position=0.1 * index,
                        velocity=0.01 * index,
                        effort=0.001 * index,
                    )
                )
                commands.append(
                    command_cls(
                        name=name,
                        sequence=index,
                        position=0.1 * index + 0.02,
                        velocity=0.01 * index + 0.03,
                        effort=0.2,
                        stiffness=10.0,
                        damping=1.0,
                    )
                )

            state_msg = joint_state_cls(header=header(timestamp_ns), joints=states)
            command_msg = joint_command_cls(header=header(timestamp_ns + 1), joints=commands)
            writer.add_message(
                state_channels[group],
                timestamp_ns,
                bytes(typestore.serialize_cdr(state_msg, "joint_msgs/msg/JointState")),
                timestamp_ns,
            )
            writer.add_message(
                command_channels[group],
                timestamp_ns + 1,
                bytes(typestore.serialize_cdr(command_msg, "joint_msgs/msg/JointCommand")),
                timestamp_ns + 1,
            )
            timestamp_ns += 1_000_000
        writer.finish()

    args = SimpleNamespace(
        raw_path=str(raw),
        output=None,
        asset_dir=str(write_test_asset_dir(tmp_path, conv)),
        derived_hz=0,
        warn_pos_error=0.2,
    )
    assert conv.convert(args) == 0

    output_mcap = tmp_path / "foxglove/a3_body_drive_debug.mcap"
    output_layout = tmp_path / "foxglove/a3_body_drive_debug.layout.json"
    assert output_mcap.is_file()
    assert output_layout.is_file()
    assert (tmp_path / "foxglove/foxglove_panel_settings.md").is_file()

    layout = json.loads(output_layout.read_text())
    assert layout["layout"]["first"] == "3D!a3"
    assert layout["layout"]["second"] == "Tab!a3-body-drive-debug"
    three_d_config = layout["configById"]["3D!a3"]
    assert three_d_config["followTf"] == "world"
    assert three_d_config["followMode"] == "follow-none"
    assert three_d_config["topics"]["/tf"]["visible"] is True
    assert three_d_config["topics"]["/joint_states"]["visible"] is True
    assert three_d_config["topics"]["/robot_description"]["visible"] is True
    assert three_d_config["layers"]["grid"]["frameId"] == "world"
    assert three_d_config["layers"]["a3_urdf"]["layerId"] == "foxglove.Urdf"
    assert three_d_config["layers"]["a3_urdf"]["sourceType"] == "topic"
    assert three_d_config["layers"]["a3_urdf"]["topic"] == "/robot_description"
    assert three_d_config["layers"]["a3_urdf"]["controlMode"] == "jointStates"
    tab_config = layout["configById"]["Tab!a3-body-drive-debug"]
    assert [tab["title"] for tab in tab_config["tabs"]] == [
        "waist pos",
        "waist tau",
        "waist vel",
        "leg pos",
        "leg tau",
        "leg vel",
        "arm pos",
        "arm tau",
        "arm vel",
    ]
    assert len([key for key in layout["configById"] if key.startswith("Plot!waist-pos-")]) == 3
    assert len([key for key in layout["configById"] if key.startswith("Plot!leg-pos-")]) == 12
    assert len([key for key in layout["configById"] if key.startswith("Plot!arm-pos-")]) == 14
    waist_pos_plot = layout["configById"]["Plot!waist-pos-0"]
    assert waist_pos_plot["paths"][0]["value"] == "/a3_debug/actual_joint_states_31.position[0]"
    assert waist_pos_plot["paths"][1]["value"] == "/a3_debug/desired_joint_states_31.position[0]"
    assert waist_pos_plot["showLegend"] is False
    assert waist_pos_plot["legendDisplay"] == "none"
    leg_pos_plot = layout["configById"]["Plot!leg-pos-19"]
    assert leg_pos_plot["paths"][0]["value"] == "/a3_debug/actual_joint_states_31.position[19]"
    assert leg_pos_plot["paths"][1]["value"] == "/a3_debug/desired_joint_states_31.position[19]"
    arm_tau_plot = layout["configById"]["Plot!arm-tau-5"]
    assert arm_tau_plot["paths"][0]["value"] == "/a3_debug/actual_joint_states_31.effort[5]"
    assert arm_tau_plot["paths"][1]["value"] == "/a3_debug/desired_joint_states_31.effort[5]"

    latest = {}
    attachment_names = set()
    with output_mcap.open("rb") as stream:
        reader = mcap_reader.make_reader(stream)
        for _schema, channel, message in reader.iter_messages():
            latest[channel.topic] = bytes(message.data)
        for attachment in reader.iter_attachments():
            attachment_names.add(attachment.name)

    required = {
        "/robot_description",
        "/tf",
        "/joint_states",
        "/a3_debug/actual_joint_states_31",
        "/a3_debug/desired_joint_states_31",
        "/a3_debug/error_joint_states_31",
        "/a3_debug/joint_error_table",
        "/a3_debug/timing",
    }
    assert required <= set(latest)
    assert "package://a3/meshes/pelvis_link.STL" in attachment_names

    desired = typestore.deserialize_cdr(latest["/a3_debug/desired_joint_states_31"], "sensor_msgs/msg/JointState")
    error = typestore.deserialize_cdr(latest["/a3_debug/error_joint_states_31"], "sensor_msgs/msg/JointState")
    urdf_state = typestore.deserialize_cdr(latest["/joint_states"], "sensor_msgs/msg/JointState")
    tf_msg = typestore.deserialize_cdr(latest["/tf"], "tf2_msgs/msg/TFMessage")
    robot_description = typestore.deserialize_cdr(latest["/robot_description"], "std_msgs/msg/String")

    assert "package://a3/meshes/pelvis_link.STL" in robot_description.data
    assert "file://" not in robot_description.data
    assert tf_msg.transforms[0].header.frame_id == "world"
    assert tf_msg.transforms[0].child_frame_id == "pelvis_link"
    assert tf_msg.transforms[0].transform.translation.x == pytest.approx(0.0)
    assert tf_msg.transforms[0].transform.rotation.z == pytest.approx(0.1)
    assert tf_msg.transforms[0].transform.rotation.w == pytest.approx(0.995)
    assert list(desired.name) == conv.A3_JOINT_NAMES_31
    assert len(desired.position) == 31
    assert len(urdf_state.name) > 31
    assert set(conv.A3_JOINT_NAMES_31) <= set(urdf_state.name)
    assert len(urdf_state.name) == len(urdf_state.position)
    assert desired.position[0] == pytest.approx(0.02)
    assert desired.velocity[0] == pytest.approx(0.03)
    assert desired.effort[0] == pytest.approx(0.2 + 10.0 * 0.02 + 1.0 * 0.03)
    assert error.position[0] == pytest.approx(0.02)
    assert error.velocity[0] == pytest.approx(0.03)
    assert error.effort[0] == pytest.approx(0.43)


def test_static_layout_template_matches_generated_layout(tmp_path):
    conv = load_converter()
    conv.write_layout(tmp_path)
    generated = json.loads((tmp_path / "a3_body_drive_debug.layout.json").read_text())
    static = json.loads(STATIC_LAYOUT.read_text())
    assert static == generated
