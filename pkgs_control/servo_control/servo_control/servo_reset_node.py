"""
Managing ROS communication for all servos in Elrik
"""

import time

import rclpy
from rclpy.node import Node

from servo_control.src.driver_waveshare import DriverWaveshare


class ServoResetNode(Node):

    def __init__(self):
        super().__init__("servo_reset_node")

        # Parameters
        self.declare_parameter(
            "config_folder_path",
            "install/wattson_description/share/wattson_description/servo_configs",
        )
        config_folder_path = (
            self.get_parameter("config_folder_path").get_parameter_value().string_value
        )

        # Configure arm servo manager
        json_files = [
            f"{config_folder_path}/servo_arm_left_params.json",
            f"{config_folder_path}/servo_arm_right_params.json",
        ]
        self.servo_driver = DriverWaveshare(json_files, 0.5)

        # Send commands
        self.servo_commands = self.servo_driver.get_default_servo_commands()
        self.servo_driver.command_servos(self.servo_commands)

        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoResetNode()
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
