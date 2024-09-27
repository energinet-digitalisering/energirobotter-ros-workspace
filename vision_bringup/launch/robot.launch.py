from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "vision_bringup"


def launch_setup(context, *args, **kwargs):

    lifecycle_nodes = [
        "face_following_node",
    ]

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("vision_bringup"), "/launch", "/vision.launch.py"]
        ),
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
        vision_launch,
        behaviour_manager_node,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
