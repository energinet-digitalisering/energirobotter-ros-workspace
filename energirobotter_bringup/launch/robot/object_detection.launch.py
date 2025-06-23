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
    use_compressed = LaunchConfiguration("use_compressed")
    run_inference_local = LaunchConfiguration("run_inference_local")

    if use_compressed.perform(context) == "true":
        image_topic = "/zed/zed_node/left/image_rect_color/compressed"
    else:
        image_topic = "/zed/zed_node/left/image_rect_color"

    image_w = 640
    image_h = 360

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
    )

    object_detection_node = Node(
        package="object_detection",
        executable="object_detection_node",
        output="screen",
        remappings=[("/camera", image_topic)],
        parameters=[
            {"box_size_multiplier": 1.5},
            {"image_w": image_w},
            {"image_h": image_h},
            {"publish_annotation": True},
            {"use_compressed": use_compressed},
        ],
        condition=IfCondition(run_inference_local),
    )

    return [
        camera_launch,
        object_detection_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_enabled",
                default_value="true",
                description="Run with or without camera.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "run_inference_local",
                default_value="true",
                description="If face_detection is run on this machine, or run separately on another",
                choices=["true", "false"],
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
