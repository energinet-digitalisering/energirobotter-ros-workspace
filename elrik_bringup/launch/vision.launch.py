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

package_name = "elrik_bringup"


def launch_setup(context, *args, **kwargs):
    start_rviz = LaunchConfiguration("rviz")
    use_mock_camera = LaunchConfiguration("use_mock_camera")
    use_compressed = LaunchConfiguration("use_compressed")

    camera_model = "zed2i"

    image_w = 640
    image_h = 360

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "vision.rviz",
        ]
    )

    servo_params = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "servo_head_params.yaml",
        ]
    )

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

        image_w = 640
        image_h = 480

    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("zed_wrapper"), "/launch", "/zed_camera.launch.py"]
        ),
        condition=IfCondition(
            PythonExpression(
                PythonExpression(f"'{use_mock_camera.perform(context)}' == 'false'")
            )
        ),  # Workaround to use "if not" condition (https://robotics.stackexchange.com/a/101015)
        launch_arguments={"camera_model": camera_model}.items(),
    )

    face_detection_node = Node(
        package="face_detection",
        executable="face_detection_node",
        output="screen",
        remappings=[("/camera", image_topic)],
        parameters=[
            {"image_w": image_w},
            {"image_h": image_h},
            {"use_compressed": use_compressed},
            {"box_size_multiplier": 0.5},
        ],
    )

    face_following_node = Node(
        package="face_following",
        executable="face_following_node",
        output="screen",
        parameters=[  # Intel Realsense Depth Camera D435i specs
            {"timer_period": 0.05},
            {"image_w": image_w},
            {"image_h": image_h},
            {"fov_w": 69},
            {"fov_h": 42},
        ],
    )

    servo_pan_node = Node(
        package="servo_control",
        executable="servo_control_node",
        name="servo_pan",
        namespace="servo_pan",
        output="screen",
        parameters=[servo_params],
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
        mock_camera_node,
        zed_camera_launch,
        face_detection_node,
        face_following_node,
        servo_pan_node,
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
