import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type = LaunchConfiguration("channel_type")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    scan_mode = LaunchConfiguration("scan_mode")

    rviz_config = os.path.join(
        get_package_share_directory("rplidar_ros"),
        "rviz",
        "rplidar_ros.rviz",
    )

    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            "channel_type": channel_type,
            "serial_port": serial_port,
            "serial_baudrate": serial_baudrate,
            "frame_id": frame_id,
            "inverted": inverted,
            "angle_compensate": angle_compensate,
            "scan_mode": scan_mode,
        }],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "channel_type",
            default_value="serial",
            description="RPLidar connection type.",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="USB serial port for the RPLidar A1.",
        ),
        DeclareLaunchArgument(
            "serial_baudrate",
            default_value="115200",
            description="Serial baudrate for the RPLidar A1.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="laser",
            description="Frame id used for the published LaserScan.",
        ),
        DeclareLaunchArgument(
            "inverted",
            default_value="false",
            description="Invert scan data.",
        ),
        DeclareLaunchArgument(
            "angle_compensate",
            default_value="true",
            description="Enable angle compensation for scan data.",
        ),
        DeclareLaunchArgument(
            "scan_mode",
            default_value="Sensitivity",
            description="RPLidar A1 scan mode.",
        ),
        rplidar_node,
        rviz_node,
    ])
