from launch import LaunchDescription
from launch_ros.actions import Node

package_name = "behaviour_manager"


def generate_launch_description():

    lifecycle_nodes = [
        "lifecycle_talker",
    ]

    lifecycle_talker_node = Node(
        package="behaviour_manager",
        executable="lifecycle_talker",
        output="screen",
    )

    behaviour_manager_node = Node(
        package="behaviour_manager",
        executable="behaviour_manager",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
        ],
    )

    return LaunchDescription(
        [
            lifecycle_talker_node,
            behaviour_manager_node,
        ]
    )
