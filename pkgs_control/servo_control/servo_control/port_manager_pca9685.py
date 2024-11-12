from adafruit_pca9685 import PCA9685
import board

import rclpy
from rclpy.node import Node

from energirobotter_interfaces.msg import ServoCommand


class PortManagerPCA9685(Node):

    def __init__(self):
        super().__init__("port_manager_pca9685")

        # Subscriptions
        self.subscription = self.create_subscription(
            ServoCommand, "pca9685/servo_command", self.callback_servo_command, 10
        )

        # Port Setup
        self.get_logger().info("Initializing I2C communication with PCA9685...")
        try:
            i2c = board.I2C()
            self.pca = PCA9685(i2c)
            self.pca.frequency = 50
            self.get_logger().info("I2C communication with PCA9685 succesful")
        except:
            self.get_logger().error("Failed to open port")
            return

    def callback_servo_command(self, msg):
        self.pca.channels[msg.servo_id].duty_cycle = msg.pwm


def main(args=None):
    rclpy.init(args=args)

    port_manager_pca9685 = PortManagerPCA9685()

    rclpy.spin(port_manager_pca9685)
    port_manager_pca9685.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
