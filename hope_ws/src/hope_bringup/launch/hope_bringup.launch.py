"""Generic HOPE bringup: motion capture -> planner.

Starts the racket planner and its ball source. Two mocap backends are
selectable via ``mocap_backend`` (both feed the same ``/poses`` contract, ball
at index 0):

* ``vrpn`` (default) — the ``vrpn_mocap`` VRPN client (configurable server
  address, default ``localhost``) plus the ``pose_to_posearray`` adapter, for
  VRPN rigs (e.g. ChingMu/Avatar Pro).
* ``optitrack`` — the vendored ``motion_capture_tracking`` NatNet driver plus
  the ``optitrack_mct_relay`` adapter (included from
  ``optitrack_hope_bridge.launch.py``), for OptiTrack/Motive rigs.
  ``mocap_server`` is the Motive PC IP; see ``docs/OPTITRACK.md``.

For testing without mocap, set ``use_fake_ball:=true`` to publish a synthetic
``/poses`` stream instead (overrides either backend).

The planner subscribes to ``poses_topic`` (a ``geometry_msgs/PoseArray`` with
the ball at ``ball_pose_index``, default 0). On the VRPN side the client
publishes one ``PoseStamped`` topic per tracker, so the ``pose_to_posearray``
node aggregates the configured tracker topic(s) into that PoseArray — set
``ball_pose_topic`` to your ball tracker's pose topic (check with
``ros2 topic list | grep vrpn``). The bundled ``client.launch.yaml`` forces
``multi_sensor: true``, and in that mode the driver names every pose topic
``pose_id_<N>`` — hence the default ``/vrpn_mocap/ball/pose_id_0`` for a
tracker named ``ball``. On the OptiTrack side the driver already names every
tracked object, so the relay maps them by name (``optitrack_relay.yaml``) and
``ball_pose_topic`` is unused. ``fake_ball_publisher`` publishes the PoseArray
form directly.

Examples::

    # Real VRPN mocap on this machine (ball tracker named "ball"):
    ros2 launch hope_bringup hope_bringup.launch.py mocap_server:=localhost

    # Real VRPN mocap on another host, different tracker topic:
    ros2 launch hope_bringup hope_bringup.launch.py mocap_server:=mocap.local \\
        mocap_port:=3883 ball_pose_topic:=/vrpn_mocap/Ball/pose_id_0

    # OptiTrack/Motive rig (mocap_server = the Motive PC IP):
    ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack \\
        mocap_server:=192.168.1.100 \\
        mocap_network_latency_ms:=<MEASURED_ONE_WAY_MS>

    # No mocap, synthetic ball for a smoke test:
    ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mocap_backend = LaunchConfiguration("mocap_backend")
    mocap_server = LaunchConfiguration("mocap_server")
    mocap_network_latency_ms = LaunchConfiguration("mocap_network_latency_ms")
    mocap_port = LaunchConfiguration("mocap_port")
    use_fake_ball = LaunchConfiguration("use_fake_ball")
    ball_pose_topic = LaunchConfiguration("ball_pose_topic")

    planner_config = Path(get_package_share_directory("hope_planner")) / "config" / "hope_planner.yaml"

    # Backend selectors. use_fake_ball overrides either backend (no mocap
    # nodes started at all); otherwise exactly one backend's nodes launch.
    fake_ball_off = ["'", use_fake_ball, "'.lower() not in ('true', '1')"]
    vrpn_selected = IfCondition(
        PythonExpression(["'", mocap_backend, "' == 'vrpn' and "] + fake_ball_off)
    )
    optitrack_selected = IfCondition(
        PythonExpression(["'", mocap_backend, "' == 'optitrack' and "] + fake_ball_off)
    )

    vrpn_client = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("vrpn_mocap"), "launch", "client.launch.yaml"])
        ),
        launch_arguments={"server": mocap_server, "port": mocap_port}.items(),
        condition=vrpn_selected,
    )

    # Real-mocap adapter (VRPN): per-tracker PoseStamped -> the planner's
    # /poses PoseArray (ball at index 0; the trigger message's header stamp is
    # passed through unmodified — with the vendored VRPN driver that is
    # ROS-side receipt time unless use_vrpn_timestamps is enabled).
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

    # Real-mocap source + adapter (OptiTrack): the vendored NatNet driver under
    # /optitrack plus optitrack_mct_relay -> /poses. start_world:=false keeps
    # this symmetric with the VRPN path (run hope_world.launch.py separately if
    # you want the static world-frame TFs).
    # The NatNet driver has no connect timeout: pointed at a host without a
    # running Motive it blocks silently in its constructor. Flag the most
    # common footgun (optitrack backend with mocap_server left at the default).
    optitrack_localhost_warning = LogInfo(
        msg="mocap_backend:=optitrack with mocap_server left at 'localhost': the "
            "NatNet driver will wait indefinitely for a Motive server on this "
            "machine. Pass mocap_server:=<MOTIVE_PC_IP> unless Motive really "
            "runs here.",
        condition=IfCondition(PythonExpression(
            ["'", mocap_backend, "' == 'optitrack' and '", mocap_server,
             "' == 'localhost' and "] + fake_ball_off)),
    )

    optitrack_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hope_bringup"), "launch", "optitrack_hope_bridge.launch.py"
            ])
        ),
        launch_arguments={
            "hostname": mocap_server,
            "start_world": "false",
            "network_latency_ms": mocap_network_latency_ms,
        }.items(),
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
        parameters=[str(planner_config)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mocap_backend", default_value="vrpn", choices=["vrpn", "optitrack"],
            description="Mocap source: 'vrpn' (vrpn_mocap client + pose_to_posearray) "
                        "or 'optitrack' (vendored motion_capture_tracking NatNet driver "
                        "+ optitrack_mct_relay). Both publish the same /poses contract."),
        DeclareLaunchArgument(
            "mocap_server", default_value="localhost",
            description="Motion-capture server IP/hostname: the VRPN server "
                        "(mocap_backend:=vrpn) or the Motive PC / NatNet server "
                        "(mocap_backend:=optitrack — always set this; the NatNet "
                        "driver blocks until a Motive server responds)."),
        DeclareLaunchArgument(
            "mocap_port", default_value="3883",
            description="VRPN motion-capture server port (vrpn backend only; NatNet "
                        "uses its own command port, fixed at 1510 in the vendored "
                        "driver, and auto-negotiates the data port)."),
        DeclareLaunchArgument(
            "mocap_network_latency_ms", default_value="0.0",
            description="OptiTrack only: measured one-way NatNet network/host receive "
                        "latency in ms for acquisition-time timestamp compensation."),
        DeclareLaunchArgument(
            "use_fake_ball", default_value="false",
            description="Publish a synthetic /poses ball stream instead of starting "
                        "any mocap backend."),
        DeclareLaunchArgument(
            "ball_pose_topic", default_value="/vrpn_mocap/ball/pose_id_0",
            description="(vrpn backend only) The ball tracker's PoseStamped topic "
                        "aggregated into /poses. The bundled VRPN client runs with "
                        "multi_sensor:=true, which names topics pose_id_<N>; a tracker "
                        "named 'ball' therefore publishes /vrpn_mocap/ball/pose_id_0. "
                        "The optitrack backend maps objects by name instead "
                        "(config/optitrack_relay.yaml)."),
        vrpn_client,
        pose_adapter,
        optitrack_localhost_warning,
        optitrack_bridge,
        fake_ball,
        planner,
    ])
