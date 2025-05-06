from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_name = "energirobotter_bringup"


def launch_setup(context, *args, **kwargs):
    config_folder_path = LaunchConfiguration("config_folder_path")

    # Servo Driver
    servo_manager_node = Node(
        package="servo_control",
        executable="servo_manager_node",
        output="screen",
        parameters=[
            {"config_folder_path": config_folder_path},
        ],
    )

    return [
        servo_manager_node,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_folder_path",
                default_value="install/wattson_description/share/wattson_description/servo_configs",
                description="Folder containing servo configs.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
