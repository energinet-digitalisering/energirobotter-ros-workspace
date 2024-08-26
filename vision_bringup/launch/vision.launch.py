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

package_name = "vision_bringup"


def launch_setup(context, *args, **kwargs):
    start_rviz = LaunchConfiguration("rviz")
    use_mock_camera = LaunchConfiguration("use_mock_camera")
    use_compressed = LaunchConfiguration("use_compressed")

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "vision.rviz",
        ]
    )

    if use_compressed.perform(context) == "true":
        image_topic = "/camera/camera/color/image_raw/compressed"
    else:
        image_topic = "/camera/camera/color/image_raw"

    mock_camera_node = Node(
        package="mock_camera",
        executable="mock_camera_node",
        output="screen",
        condition=IfCondition(use_mock_camera),
        remappings=[
            ("/camera", "/camera/camera/color/image_raw"),
        ],
    )

    real_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch", "/rs_launch.py"]
        ),
        condition=IfCondition(
            PythonExpression(
                PythonExpression(f"'{use_mock_camera.perform(context)}' == 'false'")
            )
        ),  # Workaround to use "if not" condition (https://robotics.stackexchange.com/a/101015)
    )

    face_detection_node = Node(
        package="face_detection",
        executable="face_detection_node",
        output="screen",
        remappings=[("/camera", image_topic)],
        parameters=[{"use_compressed": use_compressed}],
        parameters=[
            {"image_w": 1280},
            {"image_h": 720},
            {"use_compressed": use_compressed},
        ],
    )

    face_following_node = Node(
        package="face_following",
        executable="face_following_node",
        output="screen",
        parameters=[  # Intel Realsense Depth Camera D435i specs
            {"timer_period": 0.05},
            {"image_w": 1280},
            {"image_h": 720},
            {"fov_w": 69},
            {"fov_h": 42},
            {"servo_pos_min": 0},
            {"servo_pos_max": 180},
            {"servo_speed_max": 200.0},
            {"servo_pan_dir": 1},
            {"servo_tilt_dir": 1},
            {"servo_p_gain": 1.0},
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
        mock_camera_node,
        real_camera_launch,
        face_detection_node,
        face_following_node,
        rviz_node,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_camera",
                default_value="false",
                description="Start a mock-node instead of camera, that publishes single image with faces.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "use_compressed",
                default_value="false",
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
