from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                arguments=[
                    "install/elrik_description/share/elrik_description/urdf/phobos_generated.urdf"
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    "install/elrik_description/share/elrik_description/rviz/elrik_display.rviz",
                ],
                output="screen",
            ),
            Node(package="ik_manager", executable="ik_manager_node", output="screen"),
        ]
    )
