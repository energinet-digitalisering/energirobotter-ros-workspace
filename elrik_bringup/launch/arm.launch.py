from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
)
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "elrik_bringup"


def launch_setup(context, *args, **kwargs):

    servo_params = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "servos",
            "servo_arm_params.yaml",
        ]
    )

    # Servo Driver
    servo_driver_node = Node(
        package="servo_control",
        executable="servo_driver_waveshare",
        output="screen",
    )

    # Left Arm
    servo_left_shoulder_pitch_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="arm_left/shoulder_pitch",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    servo_right_shoulder_pitch_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="arm_right/shoulder_pitch",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    # Right Arm

    return [
        servo_driver_node,
        servo_left_shoulder_pitch_node,
        servo_right_shoulder_pitch_node,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
