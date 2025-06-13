from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "energirobotter_bringup"


def launch_setup(context, *args, **kwargs):
    camera_enabled = LaunchConfiguration("camera_enabled")
    stereo_enabled = LaunchConfiguration("stereo_enabled")
    ik_enabled = LaunchConfiguration("ik_enabled")
    ngrok_enabled = LaunchConfiguration("ngrok_enabled")
    rviz = LaunchConfiguration("rviz")

    webrtc_server_node = Node(
        package="webrtc_server_camera",
        executable="webrtc_server_camera_node",
        output="screen",
        parameters=[
            {"stereo_enabled": stereo_enabled},
        ],
        condition=IfCondition(
            camera_enabled,
        ),
    )

    ik_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare(package_name),
                "/launch",
                "/robot",
                "/ik_control.launch.py",
            ]
        ),
        condition=IfCondition(ik_enabled),
        launch_arguments={"rviz": rviz}.items(),
    )

    teleoperation_vuer_node = Node(
        package="teleoperation",
        executable="teleoperation_vuer_node",
        output="screen",
        parameters=[
            {"camera_enabled": camera_enabled},
            {"stereo_enabled": stereo_enabled},
            {"ngrok_enabled": ngrok_enabled},
        ],
    )

    return [
        webrtc_server_node,
        ik_control_launch,
        teleoperation_vuer_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_enabled",
                default_value="false",
                description="Run teleoperation with or without camera.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "stereo_enabled",
                default_value="false",
                description="Run teleoperation with or without stereo.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "ik_enabled",
                default_value="false",
                description="Run teleoperation with or without publishing IK.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "ngrok_enabled",
                default_value="false",
                description="Run teleoperation with or without ngrok.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz2 automatically with this launch file.",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
