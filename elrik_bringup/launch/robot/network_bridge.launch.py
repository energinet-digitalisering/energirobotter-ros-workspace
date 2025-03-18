from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "elrik_bringup"


def launch_setup(context, *args, **kwargs):
    camera_model = LaunchConfiguration("camera_model")
    use_compressed = LaunchConfiguration("use_compressed")

    image_topic = "/zed/zed_node/left/image_rect_color/compressed"

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare(package_name),
                "/launch",
                "/robot",
                "/camera.launch.py",
            ]
        ),
        launch_arguments={
            "camera_model": camera_model,
            "use_compressed": use_compressed,
        }.items(),
    )

    send_camera_node = Node(
        package="network_bridge",
        executable="send_camera_node",
        output="screen",
        remappings=[
            ("/camera", image_topic),
        ],
        parameters=[
            {"ip_target": "127.0.0.1"},
            {"port": 5555},
            {"use_compressed": use_compressed},
        ],
    )

    receive_tracking_node = Node(
        package="network_bridge",
        executable="receive_tracking_node",
        output="screen",
        parameters=[
            {"ip_target": "192.168.20.251"},
            {"port": 5557},
        ],
    )

    return [
        camera_launch,
        send_camera_node,
        receive_tracking_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_model",
                default_value="zedm",
                description="StereoLabs camera model.",
                choices=["zedm", "zed2i"],
            ),
            DeclareLaunchArgument(
                "use_compressed",
                default_value="true",
                description="Use compressed camera stream for faster performance.",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
