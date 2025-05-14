from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


package_name = "energirobotter_bringup"


def launch_setup(context, *args, **kwargs):
    description_package = LaunchConfiguration("description_package")

    urdf_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "urdf",
            "phobos_generated.urdf",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "rviz",
            "teleoperation.rviz",
        ]
    )

    urdf_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("urdf_launch"),
                "/launch",
                "/display.launch.py",
            ]
        ),
        launch_arguments={
            "urdf_package": description_package,
            "urdf_package_path": urdf_file,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    return [
        urdf_display_launch,
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
            OpaqueFunction(function=launch_setup),
        ]
    )
