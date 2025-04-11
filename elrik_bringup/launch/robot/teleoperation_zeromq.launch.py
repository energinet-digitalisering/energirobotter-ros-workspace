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

package_name = "elrik_bringup"


def launch_setup(context, *args, **kwargs):
    rviz = LaunchConfiguration("rviz")
    camera_enabled = LaunchConfiguration("camera_enabled")
    ik_enabled = LaunchConfiguration("ik_enabled")

    ik_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare(package_name),
                "/launch",
                "/server",
                "/ik_control.launch.py",
            ]
        ),
        condition=IfCondition(ik_enabled),
        launch_arguments={"rviz": rviz}.items(),
    )

    network_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare(package_name),
                "/launch",
                "/robot",
                "/network_bridge.launch.py",
            ]
        ),
        launch_arguments={"camera_enabled": camera_enabled}.items(),
    )

    teleoperation_zeromq_node = Node(
        package="teleoperation",
        executable="teleoperation_zeromq_node",
        output="screen",
    )

    return [
        ik_control_launch,
        network_bridge_launch,
        teleoperation_zeromq_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz2 automatically with this launch file.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "camera_enabled",
                default_value="false",
                description="Run teleoperation with or without camera.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "ik_enabled",
                default_value="false",
                description="Run teleoperation with or without publishing IK.",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
