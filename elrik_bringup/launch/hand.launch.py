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
            "servo_hand_params.yaml",
        ]
    )

    # Servo Driver
    servo_driver_node = Node(
        package="servo_control",
        executable="servo_driver_pca9685",
        output="screen",
    )

    # Left Hand
    servo_left_pinky_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_left/pinky",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    servo_left_ring_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_left/ring",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    servo_left_middle_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_left/middle",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    servo_left_index_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_left/index",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    servo_left_thumb_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_left/thumb",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    # Right Hand
    servo_right_pinky_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_right/pinky",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    servo_right_ring_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_right/ring",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    servo_right_middle_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_right/middle",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    servo_right_index_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_right/index",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    servo_right_thumb_node = Node(
        package="servo_control",
        executable="servo_control_node",
        namespace="hand_right/thumb",
        name="servo",
        output="screen",
        parameters=[servo_params],
    )

    return [
        servo_driver_node,
        servo_left_pinky_node,
        servo_left_ring_node,
        servo_left_middle_node,
        servo_left_index_node,
        servo_left_thumb_node,
        servo_right_pinky_node,
        servo_right_ring_node,
        servo_right_middle_node,
        servo_right_index_node,
        servo_right_thumb_node,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
