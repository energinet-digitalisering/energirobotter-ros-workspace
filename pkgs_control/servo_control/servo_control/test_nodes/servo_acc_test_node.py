from itertools import cycle

import rclpy
from rclpy.node import Node

from ..src.SCServo_Python.scservo_sdk import PortHandler, sms_sts

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200


class ServoAccTestNode(Node):

    def __init__(self):
        super().__init__("servo_acc_test_node")

        # Parameters
        self.declare_parameter("control_frequency", 0.1)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        # Timers
        self.timer = self.create_timer(
            1.0 / self.control_frequency, self.callback_timer
        )

        # Setup driver
        self.driver_object = self.setup_driver()

        if self.driver_object is None:
            self.destroy_node()

        # Node variables
        self.servo_id = 18
        self.pwm_min = 0
        self.pwm_max = 4094
        self.pwm_current = 0

        self.pwm_iterator = cycle([self.pwm_min, self.pwm_max])

    def setup_driver(self):

        self.get_logger().info("Initializing serial communication with Waveshare...")

        try:
            self.port_handler = PortHandler(PORT)
            packet_handler = sms_sts(self.port_handler)

            if not self.port_handler.openPort():
                self.get_logger().error("Failed to open port")
                return None

            if not self.port_handler.setBaudRate(BAUDRATE):
                self.get_logger().error("Failed to set baud rate")
                return None

            self.get_logger().info("Serial communication successful")
            return packet_handler

        except Exception as e:
            self.get_logger().error(f"Failed to open port: {e}")
            return None

    def callback_timer(self):

        pwm = next(self.pwm_iterator)
        self.get_logger().info(f"PWM: {pwm}")

        scs_comm_result, scs_error = self.driver_object.WritePosEx(
            self.servo_id, pwm, SCS_MOVING_SPEED := 0, SCS_MOVING_ACC := 4
        )


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoAccTestNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
