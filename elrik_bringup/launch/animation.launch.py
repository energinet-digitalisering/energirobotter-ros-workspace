import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from launch_ros.actions import Node


package_name = "elrik_bringup"


def launch_setup(context, *args, **kwargs):

    package_share_dir = get_package_share_directory(package_name)
    animation_file_path = os.path.join(package_share_dir, "animations", "recording.csv")

    lifecycle_nodes = [
        "animation_player_node",
    ]

    animation_player_node = Node(
        package="animation_player",
        executable="animation_player_node",
        output="screen",
        parameters=[
            {"csv_file_path": animation_file_path},
        ],
    )

    behaviour_manager_node = Node(
        package="behaviour_manager",
        executable="behaviour_manager",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
        ],
    )

    return [
        animation_player_node,
        behaviour_manager_node,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
