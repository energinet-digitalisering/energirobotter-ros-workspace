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
    rviz = LaunchConfiguration("rviz")
    camera_enabled = LaunchConfiguration("camera_enabled")
    ik_enabled = LaunchConfiguration("ik_enabled")

    image_topic_left = "/zed/zed_node/left/image_rect_color/compressed"
    image_topic_right = "/zed/zed_node/right/image_rect_color/compressed"

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

    teleoperation_vuer_node = Node(
        package="teleoperation",
        executable="teleoperation_vuer_node",
        output="screen",
        remappings=[
            ("/image_left", image_topic_left),
            ("/image_right", image_topic_right),
        ],
        parameters=[
            {"camera_enabled": camera_enabled},
        ],
    )

    return [
        ik_control_launch,
        teleoperation_vuer_node,
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
