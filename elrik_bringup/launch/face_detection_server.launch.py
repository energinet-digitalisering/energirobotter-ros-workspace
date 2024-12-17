from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "elrik_bringup"


def launch_setup(context, *args, **kwargs):
    start_rviz = LaunchConfiguration("rviz")
    use_mock_camera = LaunchConfiguration("use_mock_camera")
    use_compressed = LaunchConfiguration("use_compressed")

    image_w = 640
    image_h = 360

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "rviz",
            "vision.rviz",
        ]
    )

    if use_compressed.perform(context) == "true":
        image_topic = "/zed/zed_node/left/image_rect_color/compressed"
    else:
        image_topic = "/zed/zed_node/left/image_rect_color"

    if use_mock_camera.perform(context) == "webcam":
        # Overwrite node to webcam
        image_w = 640
        image_h = 480

    face_detection_node = Node(
        package="face_detection",
        executable="face_detection_node",
        output="screen",
        remappings=[("/camera", image_topic)],
        parameters=[
            {"image_w": image_w},
            {"image_h": image_h},
            {"use_compressed": use_compressed},
            {"box_size_multiplier": 1.5},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    return [
        face_detection_node,
        rviz_node,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_camera",
                default_value="false",
                description="Start a mock-node instead of camera, that publishes single image with faces.",
                choices=["true", "false", "webcam"],
            ),
            DeclareLaunchArgument(
                "use_compressed",
                default_value="true",
                description="Use compressed camera stream for faster performance. OBS: should be False when using mock camera.",
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
