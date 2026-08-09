"""Generic HOPE bringup: motion capture -> planner.

Starts the racket planner and its ball source. Two mocap backends are
selectable via ``mocap_backend`` (both feed the same ``/poses`` contract, ball
at index 0):

* ``vrpn`` (default) — the ``pose_to_posearray`` adapter for an independently
  built and launched ``VRPN2ROS2`` workspace (e.g. ChingMu/Avatar Pro).
* ``optitrack`` — the ``optitrack_mct_relay`` adapter (included from
  ``optitrack_hope_bridge.launch.py``), for an independently built and
  launched ``NatNet2ROS2`` workspace. See ``docs/OPTITRACK.md``.

For testing without mocap, set ``use_fake_ball:=true`` to publish a synthetic
``/poses`` stream instead (overrides either backend).

The planner subscribes to ``poses_topic`` (a ``geometry_msgs/PoseArray`` with
the ball at ``ball_pose_index``, default 0). On the VRPN side the external
client publishes one ``PoseStamped`` topic per tracker, so ``pose_to_posearray``
node aggregates the configured tracker topic(s) into that PoseArray — set
``ball_pose_topic`` to your ball tracker's pose topic (check with
``ros2 topic list | grep vrpn``). The bundled ``client.launch.yaml`` forces
``multi_sensor: true``, and in that mode the driver names every pose topic
``pose_id_<N>`` — hence the default ``/vrpn_mocap/Ball/pose_id_0`` for a
tracker named ``Ball``. On the OptiTrack side the driver already names every
tracked object, so the relay maps them by name (``optitrack_relay.yaml``) and
``ball_pose_topic`` is unused. ``fake_ball_publisher`` publishes the PoseArray
form directly.

Examples::

    # Real VRPN mocap (start VRPN2ROS2 independently first):
    ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=vrpn \\
        ball_pose_topic:=/vrpn_mocap/Ball/pose_id_0

    # OptiTrack/Motive rig (start NatNet2ROS2 independently first):
    ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack

    # No mocap, synthetic ball for a smoke test:
    ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mocap_backend = LaunchConfiguration("mocap_backend")
    use_fake_ball = LaunchConfiguration("use_fake_ball")
    ball_pose_topic = LaunchConfiguration("ball_pose_topic")
    planner_fit_window = LaunchConfiguration("planner_fit_window")

    planner_config = (
        Path(get_package_share_directory("hope_planner"))
        / "config"
        / "hope_planner.yaml"
    )

    # Backend selectors. use_fake_ball overrides either backend (no mocap
    # nodes started at all); otherwise exactly one backend's nodes launch.
    fake_ball_off = ["'", use_fake_ball, "'.lower() not in ('true', '1')"]
    vrpn_selected = IfCondition(
        PythonExpression(["'", mocap_backend, "' == 'vrpn' and "] + fake_ball_off)
    )
    optitrack_selected = IfCondition(
        PythonExpression(["'", mocap_backend, "' == 'optitrack' and "] + fake_ball_off)
    )

    # Real-mocap adapter (VRPN): the independently launched VRPN2ROS2 publishes
    # per-tracker PoseStamped; this node maps them into the planner's
    # /poses PoseArray (ball at index 0; the trigger message's header stamp is
    # passed through unmodified).
    # NOTE the nested list [[ball_pose_topic]]: launch_ros collapses a FLAT list of
    # substitutions into one concatenated string, which would violate the node's
    # STRING_ARRAY parameter type; the list-of-lists form evaluates to a string array.
    pose_adapter = Node(
        package="hope_bringup",
        executable="pose_to_posearray",
        name="pose_to_posearray",
        output="screen",
        parameters=[{"input_topics": [[ball_pose_topic]], "trigger_index": 0}],
        condition=vrpn_selected,
    )

    # Real-mocap adapter (OptiTrack): NatNet2ROS2 is an independent workspace
    # and must already be publishing /optitrack/poses. This include starts only
    # the HOPE relay. start_world:=false keeps the path symmetric with VRPN.
    optitrack_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hope_bringup"), "launch", "optitrack_hope_bridge.launch.py"
            ])
        ),
        launch_arguments={"start_world": "false"}.items(),
        condition=optitrack_selected,
    )

    fake_ball = Node(
        package="hope_bringup",
        executable="fake_ball_publisher",
        name="fake_ball_publisher",
        output="screen",
        condition=IfCondition(use_fake_ball),
    )

    planner = Node(
        package="hope_planner",
        executable="hope_planner_node",
        name="hope_planner",
        output="screen",
        parameters=[
            str(planner_config),
            {"fit_window": ParameterValue(planner_fit_window, value_type=int)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mocap_backend", default_value="vrpn", choices=["vrpn", "optitrack"],
            description="Mocap source: 'vrpn' (relay for independent VRPN2ROS2) "
                        "or 'optitrack' (relay for an independently launched "
                        "NatNet2ROS2 adapter). Both publish the same /poses contract."),
        DeclareLaunchArgument(
            "use_fake_ball", default_value="false",
            description="Publish a synthetic /poses ball stream instead of starting "
                        "any mocap backend."),
        DeclareLaunchArgument(
            "ball_pose_topic", default_value="/vrpn_mocap/Ball/pose_id_0",
            description="(vrpn backend only) The ball tracker's PoseStamped topic "
                        "aggregated into /poses. The external VRPN2ROS2 client runs with "
                        "multi_sensor:=true, which names topics pose_id_<N>; a tracker "
                        "named 'Ball' therefore publishes /vrpn_mocap/Ball/pose_id_0. "
                        "The optitrack backend maps objects by name instead "
                        "(config/optitrack_relay.yaml)."),
        DeclareLaunchArgument(
            "planner_fit_window",
            default_value=PythonExpression(
                ["19 if '", mocap_backend, "' == 'optitrack' else 21"]
            ),
            description="Planner velocity-fit samples. The default preserves an "
                        "approximately 100 ms window for NatNet2ROS2's 180 Hz "
                        "OptiTrack output or VRPN2ROS2's 200 Hz output. "
                        "Override when changing the adapter output rate."),
        pose_adapter,
        optitrack_bridge,
        fake_ball,
        planner,
    ])
