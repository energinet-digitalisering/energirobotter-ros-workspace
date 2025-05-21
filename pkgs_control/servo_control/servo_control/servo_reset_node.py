"""
Managing ROS communication for all servos in Elrik
"""

import rclpy
from rclpy.node import Node

from servo_control.src.elrik_driver_arms import DriverWaveshare


class ServoResetNode(Node):

    def __init__(self):
        super().__init__("servo_reset_node")

        # Parameters
        self.declare_parameter("control_frequency", 0.5)
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

        # Timers
        self.timer = self.create_timer(self.control_frequency, self.callback_timer)

        # Node variables

        # Configure arm servo manager
        json_files_arms = [
            f"{config_folder_path}/servo_arm_left_params.json",
            f"{config_folder_path}/servo_arm_right_params.json",
        ]
        self.driver_arms = DriverWaveshare(json_files_arms, self.control_frequency)

        self.servo_commands_arms = {}
        for servo in self.driver_arms.get_default_servo_commands().keys():
            self.servo_commands_arms[servo] = 1

    def callback_timer(self):
        self.driver_arms.command_servos(self.servo_commands_arms)


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoResetNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
