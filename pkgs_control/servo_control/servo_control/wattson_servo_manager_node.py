"""
Managing ROS communication for all servos in Wattson
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from servo_control.src.driver_waveshare import DriverWaveshare


class ServoManagerNode(Node):

    def __init__(self):
        super().__init__("servo_manager_node")

        # Parameters
        self.declare_parameter("control_frequency", 10.0)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        self.declare_parameter(
            "config_folder_path",
            "install/energirobotter_bringup/share/energirobotter_bringup/config/servos",
        )
        config_folder_path = (
            self.get_parameter("config_folder_path").get_parameter_value().string_value
        )

        # Subscriptions
        self.sub_joints_arms = self.create_subscription(
            JointState, "/joint_states", self.callback_joints_arms, 1
        )
        self.sub_joints_hands = self.create_subscription(
            JointState, "/joint_states_hands", self.callback_joints_hands, 1
        )

        # Publishers

        # DEBUG
        self.pub_speeds = self.create_publisher(JointState, "/log_speeds", 10)
        # DEBUG END

        # Timers
        self.timer = self.create_timer(
            1.0 / self.control_frequency, self.callback_timer
        )

        # Configure servo manager
        json_files = [
            f"{config_folder_path}/servo_arm_left_params.json",
            f"{config_folder_path}/servo_arm_right_params.json",
        ]
        self.servo_driver = DriverWaveshare(json_files, self.control_frequency)
        self.servo_commands = self.servo_driver.get_default_servo_commands()

        # Node variables
        self.servo_commands_arms = {}
        self.servo_commands_hands = {}

    def callback_joints_arms(self, msg):
        self.servo_commands_arms = dict(zip(msg.name, np.rad2deg(msg.position)))

    def callback_joints_hands(self, msg):
        self.servo_commands_hands = dict(zip(msg.name, np.rad2deg(msg.position)))

        # Map angles to servo range for fingers
        for servo_name in self.servo_commands_hands:
            servo = self.servo_driver.servos[servo_name]
            command = self.servo_commands_hands[servo_name]

            angle_mapped = self.servo_driver.map_finger_to_servo(servo, command)
            self.servo_commands_hands[servo_name] = angle_mapped

    def callback_timer(self):
        # Combine command dicts into one
        self.servo_commands = self.servo_commands_arms | self.servo_commands_hands

        # Update servos
        self.servo_driver.update_feedback()
        self.servo_driver.command_servos(self.servo_commands)


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoManagerNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
