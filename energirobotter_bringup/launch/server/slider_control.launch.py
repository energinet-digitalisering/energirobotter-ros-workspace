from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


package_name = "energirobotter_bringup"


def launch_setup(context, *args, **kwargs):

    urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("elrik_description"),
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
            "urdf_package": "elrik_description",
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
            OpaqueFunction(function=launch_setup),
        ]
    )
