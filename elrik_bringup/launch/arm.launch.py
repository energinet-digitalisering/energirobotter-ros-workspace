from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

package_name = "elrik_bringup"


def launch_setup(context, *args, **kwargs):

    # Servo Driver
    servo_manager_node = Node(
        package="servo_control",
        executable="servo_manager_node",
        output="screen",
    )

    return [
        servo_manager_node,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
