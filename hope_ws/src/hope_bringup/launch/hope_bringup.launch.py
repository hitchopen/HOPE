"""Generic HOPE PingPong bringup: motion capture -> planner.

Starts the racket planner and its ball source. By default it launches the
``vrpn_mocap`` VRPN client (configurable server address, default ``localhost``)
plus the ``pose_to_posearray`` adapter so the planner runs against a real
motion-capture stream. For testing without mocap, set ``use_fake_ball:=true``
to publish a synthetic ``/poses`` stream instead.

The planner subscribes to ``poses_topic`` (a ``geometry_msgs/PoseArray`` with
the ball at ``ball_pose_index``, default 0). The VRPN client publishes one
``PoseStamped`` topic per tracker, so the ``pose_to_posearray`` node aggregates
the configured tracker topic(s) into that PoseArray — set ``ball_pose_topic`` to
your ball tracker's pose topic (check with ``ros2 topic list | grep vrpn``; with
``multi_sensor: true`` VRPN configs the topic carries an ``_id_<N>`` suffix).
``fake_ball_publisher`` publishes the PoseArray form directly.

Examples::

    # Real mocap on this machine (ball tracker named "ball"):
    ros2 launch hope_bringup hope_bringup.launch.py mocap_server:=localhost

    # Real mocap on another host, different tracker topic:
    ros2 launch hope_bringup hope_bringup.launch.py mocap_server:=mocap.local \\
        mocap_port:=3883 ball_pose_topic:=/vrpn_mocap/Ball/pose_id_0

    # No mocap, synthetic ball for a smoke test:
    ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mocap_server = LaunchConfiguration("mocap_server")
    mocap_port = LaunchConfiguration("mocap_port")
    use_fake_ball = LaunchConfiguration("use_fake_ball")
    ball_pose_topic = LaunchConfiguration("ball_pose_topic")

    planner_config = Path(get_package_share_directory("hope_planner")) / "config" / "hope_planner.yaml"

    vrpn_client = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("vrpn_mocap"), "launch", "client.launch.yaml"])
        ),
        launch_arguments={"server": mocap_server, "port": mocap_port}.items(),
        condition=UnlessCondition(use_fake_ball),
    )

    # Real-mocap adapter: per-tracker PoseStamped -> the planner's /poses PoseArray
    # (ball at index 0, capture stamps passed through).
    # NOTE the nested list [[ball_pose_topic]]: launch_ros collapses a FLAT list of
    # substitutions into one concatenated string, which would violate the node's
    # STRING_ARRAY parameter type; the list-of-lists form evaluates to a string array.
    pose_adapter = Node(
        package="hope_bringup",
        executable="pose_to_posearray",
        name="pose_to_posearray",
        output="screen",
        parameters=[{"input_topics": [[ball_pose_topic]], "trigger_index": 0}],
        condition=UnlessCondition(use_fake_ball),
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
        parameters=[str(planner_config)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mocap_server", default_value="localhost",
            description="VRPN motion-capture server IP/hostname."),
        DeclareLaunchArgument(
            "mocap_port", default_value="3883",
            description="VRPN motion-capture server port."),
        DeclareLaunchArgument(
            "use_fake_ball", default_value="false",
            description="Publish a synthetic /poses ball stream instead of starting vrpn_mocap."),
        DeclareLaunchArgument(
            "ball_pose_topic", default_value="/vrpn_mocap/ball/pose",
            description="The ball tracker's PoseStamped topic aggregated into /poses "
                        "(with multi_sensor VRPN configs this may carry an _id_<N> suffix)."),
        vrpn_client,
        pose_adapter,
        fake_ball,
        planner,
    ])
