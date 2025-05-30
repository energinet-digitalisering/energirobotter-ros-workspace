"""
Managing ROS communication for all servos in Elrik
"""

import rclpy
from rclpy.node import Node

from servo_control.src.driver_waveshare import DriverWaveshare


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
            "install/wattson_description/share/wattson_description/servo_configs",
        )
        config_folder_path = (
            self.get_parameter("config_folder_path").get_parameter_value().string_value
        )

        # Timers
        self.timer = self.create_timer(self.control_frequency, self.callback_timer)

        # Node variables

        # Configure arm servo manager
        json_files = [
            f"{config_folder_path}/servo_arm_left_params.json",
            f"{config_folder_path}/servo_arm_right_params.json",
        ]

        self.servo_driver = DriverWaveshare(json_files, self.control_frequency)
        self.servo_commands = self.servo_driver.get_default_servo_commands()

    def callback_timer(self):
        self.servo_driver.command_servos(self.servo_commands)


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoResetNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
