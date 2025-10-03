from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "energirobotter_bringup"


def launch_setup(context, *args, **kwargs):
    description_package = LaunchConfiguration("description_package")
    base_link = LaunchConfiguration("base_link")
    tip_link = LaunchConfiguration("tip_link")
    step_size = LaunchConfiguration("step_size")

    urdf_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", "phobos_generated.urdf"]
    ).perform(context)

    robot_description = Path(urdf_file).read_text()

    reachability_map_generator = Node(
        package="kinematics_manager",
        executable="reachability_map_generator",
        parameters=[
            {"robot_description": robot_description},
            {"base_link": base_link.perform(context)},
            {"tip_link": tip_link.perform(context)},
            {"step_size": float(step_size.perform(context))},
        ],
        output="screen",
    )

    return [
        reachability_map_generator,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "description_package",
                default_value="wattson_description",
                description="Package in workspace that contains robot URDF description.",
                choices=["elrik_description", "wattson_description"],
            ),
            DeclareLaunchArgument(
                "base_link",
                default_value="link_torso",
                description="Base link for kinematics calculations.",
            ),
            DeclareLaunchArgument(
                "tip_link",
                default_value="link_left_hand",
                description="Tip link (end-effector) to analyze.",
            ),
            DeclareLaunchArgument(
                "step_size",
                default_value="0.02",
                description="Resolution of the workspace grid in meters.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
