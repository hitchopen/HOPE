"""Standalone NatNet -> ROS 2 adapter launch for HOPE deployments.

This workspace owns the NatNet connection and publishes one filtered, raw
``/optitrack/poses`` stream containing only ``Ball``, ``P1``, and ``P2`` when
available. A selected valid frame containing none is emitted as an empty-array
heartbeat. HOPE-specific conversion to ``/poses`` and TF is performed
separately by ``hope_bringup/optitrack_mct_relay``.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_path = (
        Path(get_package_share_directory("motion_capture_tracking"))
        / "config"
        / "hope_optitrack.yaml"
    )

    hostname = LaunchConfiguration("hostname")
    interface_ip = LaunchConfiguration("interface_ip")
    mocap_type = LaunchConfiguration("mocap_type")
    header_time = LaunchConfiguration("header_time")
    output_rate_hz = LaunchConfiguration("output_rate_hz")
    network_latency_ms = LaunchConfiguration("network_latency_ms")
    publish_p1_markers = LaunchConfiguration("publish_p1_markers")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hostname",
                description="REQUIRED: Motive/NatNet server IPv4 address.",
            ),
            DeclareLaunchArgument(
                "interface_ip",
                default_value="",
                description=(
                    "IPv4 address assigned to this computer's wired "
                    "Motive-network NIC. Required for live multicast; an empty "
                    "value is accepted only for mock/unicast operation."
                ),
            ),
            DeclareLaunchArgument(
                "mocap_type",
                default_value="optitrack",
                description="libmotioncapture backend: optitrack or mock.",
            ),
            DeclareLaunchArgument(
                "header_time",
                default_value="camera_utc",
                choices=["camera_utc", "ros"],
                description="Timestamp mode. Keep camera_utc for moving/cross-sensor "
                "alignment. ros is an explicit receipt-time diagnostic fallback "
                "for Motive versions without NatNet echo support.",
            ),
            DeclareLaunchArgument(
                "output_rate_hz",
                default_value="200.0",
                description="Maximum ROS 2 output rate in Hz. Set 0 to publish "
                "every valid NatNet source frame.",
            ),
            DeclareLaunchArgument(
                "network_latency_ms",
                default_value="0.0",
                description="Legacy ros_latency_compensated mode only; camera_utc "
                    "uses measured NatNet echo synchronization.",
            ),
            DeclareLaunchArgument(
                "publish_p1_markers",
                default_value="false",
                description=(
                    "Per-run calibration support: publish atomic P1 ModelDef and "
                    "same-frame labeled-marker samples on "
                    "/optitrack/rigid_body_markers. Motive Labeled Markers "
                    "must be ON. The laptop bringup keeps this local publisher "
                    "available; recalculation still occurs only on PREPARE."
                ),
            ),
            Node(
                package="motion_capture_tracking",
                executable="motion_capture_tracking_node",
                namespace="optitrack",
                name="motion_capture_tracking_node",
                output="screen",
                # The driver exits non-zero after a bounded stream timeout.
                # Restart here as well as in managed venue supervisors so the
                # documented standalone launch recovers when Motive resumes.
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    str(config_path),
                    {
                        "hostname": hostname,
                        "interface_ip": interface_ip,
                        "type": mocap_type,
                        "topics.header_time": header_time,
                        "topics.output_rate_hz": ParameterValue(
                            output_rate_hz, value_type=float
                        ),
                        "topics.network_latency_ms": ParameterValue(
                            network_latency_ms, value_type=float
                        ),
                        "topics.rigid_body_markers.enabled": ParameterValue(
                            publish_p1_markers, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
