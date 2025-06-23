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
    camera_model = LaunchConfiguration("camera_model")
    camera_enabled = LaunchConfiguration("camera_enabled")
    use_compressed = LaunchConfiguration("use_compressed")
    ip_target = LaunchConfiguration("ip_target")

    image_topic_left = "/zed/zed_node/left/image_rect_color/compressed"
    image_topic_right = "/zed/zed_node/right/image_rect_color/compressed"

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare(package_name),
                "/launch",
                "/robot",
                "/camera.launch.py",
            ]
        ),
        condition=IfCondition(camera_enabled),
        launch_arguments={
            "camera_model": camera_model,
            "use_compressed": use_compressed,
        }.items(),
    )

    send_camera_node_left = Node(
        package="network_bridge",
        executable="send_camera_node",
        output="screen",
        name="send_camera_left_node",
        remappings=[
            ("/camera", image_topic_left),
        ],
        parameters=[
            {"ip_target": "0.0.0.0"},
            {"port": 5555},
            {"use_compressed": use_compressed},
        ],
        condition=IfCondition(camera_enabled),
    )

    send_camera_right_node = Node(
        package="network_bridge",
        executable="send_camera_node",
        output="screen",
        name="send_camera_right_node",
        remappings=[
            ("/camera", image_topic_right),
        ],
        parameters=[
            {"ip_target": "0.0.0.0"},
            {"port": 5556},
            {"use_compressed": use_compressed},
        ],
        condition=IfCondition(camera_enabled),
    )

    receive_tracking_node = Node(
        package="network_bridge",
        executable="receive_tracking_node",
        output="screen",
        parameters=[
            {"ip_target": ip_target},
            {"port": 5557},
        ],
    )

    return [
        camera_launch,
        send_camera_node_left,
        send_camera_right_node,
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
                "camera_enabled",
                default_value="false",
                description="Run teleoperation with or without camera.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "use_compressed",
                default_value="true",
                description="Use compressed camera stream for faster performance.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "ip_target",
                default_value="localhost",
                description="IP Address of teleoperation device.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
