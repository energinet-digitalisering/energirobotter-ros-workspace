from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "energirobotter_bringup"


def launch_setup(context, *args, **kwargs):
    use_mock_camera = LaunchConfiguration("use_mock_camera")
    use_compressed = LaunchConfiguration("use_compressed")
    camera_model = LaunchConfiguration("camera_model")

    zed_camera_params = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "zed_camera",
            "optimised.yaml",
        ]
    ).perform(context)

    if use_compressed.perform(context) == "true":
        image_topic = "/zed/zed_node/left/image_rect_color/compressed"
    else:
        image_topic = "/zed/zed_node/left/image_rect_color"

    mock_camera_node = Node(
        package="mock_camera",
        executable="photo_pub_node",
        output="screen",
        condition=IfCondition(use_mock_camera),
        remappings=[
            ("/camera", image_topic),
        ],
        parameters=[
            {"use_compressed": use_compressed},
        ],
    )

    if use_mock_camera.perform(context) == "webcam":
        # Overwrite node to webcam
        mock_camera_node = Node(
            package="mock_camera",
            executable="webcam_pub_node",
            output="screen",
            remappings=[
                ("/camera", image_topic),
            ],
        )

    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("zed_wrapper"), "/launch", "/zed_camera.launch.py"]
        ),
        condition=IfCondition(
            PythonExpression(
                PythonExpression(f"'{use_mock_camera.perform(context)}' == 'false'")
            )
        ),  # Workaround to use "if not" condition (https://robotics.stackexchange.com/a/101015)
        launch_arguments={
            "camera_model": camera_model,
            "config_path": zed_camera_params,
        }.items(),
    )

    return [
        mock_camera_node,
        zed_camera_launch,
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
                "camera_model",
                default_value="zedm",
                description="StereoLabs camera model.",
                choices=["zedm", "zed2i"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
