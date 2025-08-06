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
    camera_source = LaunchConfiguration("camera_source")
    stereo_enabled = LaunchConfiguration("stereo_enabled")
    ik_enabled = LaunchConfiguration("ik_enabled")
    rviz = LaunchConfiguration("rviz")

    launch_nodes = []

    image_topic_left = "/zed/zed_node/left/image_rect_color/compressed"
    image_topic_right = "/zed/zed_node/right/image_rect_color/compressed"

    if camera_source.perform(context) == "ngrok":
        webrtc_server_node = Node(
            package="webrtc_server_camera",
            executable="webrtc_server_camera_node",
            output="screen",
            parameters=[
                {"stereo_enabled": stereo_enabled},
            ],
        )
        launch_nodes.append(webrtc_server_node)

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
        remappings=[
            ("/image_left", image_topic_left),
            ("/image_right", image_topic_right),
        ],
        parameters=[
            {"camera_source": camera_source},
        ],
    )

    return [
        ik_control_launch,
        teleoperation_vuer_node,
    ] + launch_nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_source",
                default_value="",
                description="Choose alternative camera that Zed, ex. webcam or a static image (mock).",
                choices=["", "ros", "server", "ngrok"],
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
                "rviz",
                default_value="false",
                description="Start RViz2 automatically with this launch file.",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
