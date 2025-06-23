"""
Node to test servos connected to a PCA9685.
"""

from adafruit_pca9685 import PCA9685
import board
import busio

import rclpy
from rclpy.node import Node

from energirobotter_interfaces.msg import ServoCommand


class ServoDriverPCA9685(Node):

    def __init__(self):
        super().__init__("servo_driver_pca9685")

        # Subscriptions
        self.subscription = self.create_subscription(
            ServoCommand, "/pca9685/servo_command", self.callback_servo_command, 10
        )

        # Port Setup
        self.get_logger().info("Initializing I2C communication with PCA9685...")
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            # i2c = busio.I2C(board.SCL_1, board.SDA_1)
            self.pca = PCA9685(i2c, address=0x40)
            self.pca.frequency = 50
            self.get_logger().info("I2C communication with PCA9685 succesful")
        except Exception as e:
            self.get_logger().error(f"Failed to open port: {e}, shutting down node...")
            self.destroy_node()

    def callback_servo_command(self, msg):
        self.pca.channels[msg.servo_id].duty_cycle = msg.pwm


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoDriverPCA9685()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
