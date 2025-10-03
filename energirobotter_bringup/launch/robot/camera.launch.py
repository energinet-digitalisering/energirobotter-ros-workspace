from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "energirobotter_bringup"


def launch_setup(context, *args, **kwargs):
    camera_mode = LaunchConfiguration("camera_mode")
    camera_model = LaunchConfiguration("camera_model")
    rotate = LaunchConfiguration("rotate").perform(context)
    use_compressed = LaunchConfiguration("use_compressed")

    launch_nodes = []

    if camera_mode.perform(context) == "camera":

        zed_camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("zed_wrapper"), "/launch", "/zed_camera.launch.py"]
            ),
            launch_arguments={
                "camera_model": camera_model,
            }.items(),
        )

        launch_nodes.append(zed_camera_launch)

    if use_compressed.perform(context) == "true":
        image_topic = "/zed/zed_node/left/image_rect_color/compressed"
    else:
        image_topic = "/zed/zed_node/left/image_rect_color"

    if rotate:
        angles_deg = float(rotate)
        suffix = "/compressed" if use_compressed else ""

        image_rotate_node = Node(
            package="image_processing",
            executable="image_rotate_node",
            output="screen",
            remappings=[
                ("image", image_topic),
                ("image_rotated", image_topic + "/rotated" + suffix),
            ],
            parameters=[
                {"rotation": angles_deg},
                {"use_compressed": use_compressed},
            ],
        )

        launch_nodes.append(image_rotate_node)

    # --- Test cameras ---

    if camera_mode.perform(context) == "webcam":
        webcam_node = Node(
            package="mock_camera",
            executable="webcam_pub_node",
            output="screen",
            remappings=[
                ("/camera", image_topic),
            ],
        )

        launch_nodes.append(webcam_node)

    if camera_mode.perform(context) == "mock":

        mock_camera_node = Node(
            package="mock_camera",
            executable="photo_pub_node",
            output="screen",
            remappings=[
                ("/camera", image_topic),
            ],
            parameters=[
                {"use_compressed": use_compressed},
            ],
        )

        launch_nodes.append(mock_camera_node)

    return launch_nodes


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_mode",
                default_value="camera",
                description="Choose alternative camera that Zed, ex. webcam or a static image (mock).",
                choices=["camera", "mock", "webcam"],
            ),
            DeclareLaunchArgument(
                "camera_model",
                default_value="zedm",
                description="StereoLabs camera model.",
                choices=["zedm", "zed2i"],
            ),
            DeclareLaunchArgument(
                "rotate",
                default_value="",
                description="If set, a new topic will be created with the image stream rotated by specified amount of angles (degrees).",
            ),
            DeclareLaunchArgument(
                "use_compressed",
                default_value="true",
                description="The mock camera will create an image topic matching the name of the Zed cameras. Set this if compressed topic is needed with the mock camera.",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
