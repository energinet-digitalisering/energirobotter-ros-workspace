from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                arguments=[
                    "install/elrik_description/share/elrik_description/urdf/elrik.urdf"
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    "install/elrik_description/share/elrik_description/config/elrik.rviz",
                ],
                output="screen",
            ),
            Node(
                package="ik_manager", executable="ik_visualizer_node", output="screen"
            ),
        ]
    )
