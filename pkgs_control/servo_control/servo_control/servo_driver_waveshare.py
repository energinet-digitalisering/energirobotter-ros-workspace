import rclpy
from rclpy.node import Node

from .src.SCServo_Python.scservo_sdk import PortHandler, sms_sts, scservo_def

from energirobotter_interfaces.msg import ServoCommand


class ServoDriverWaveshare(Node):

    def __init__(self):
        super().__init__("servo_driver_waveshare")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        port = self.get_parameter("port").get_parameter_value().string_value

        self.declare_parameter("baudrate", 115200)
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        # Subscriptions
        self.subscription = self.create_subscription(
            ServoCommand, "waveshare/servo_command", self.callback_servo_command, 10
        )

        # Port Setup
        self.get_logger().info("Initializing serial communication with Waveshare...")
        try:
            self.port_handler = PortHandler(port)
            self.packet_handler = sms_sts(self.port_handler)

            if not self.port_handler.openPort():
                self.get_logger().error("Failed to open port, shutting down node...")
                self.destroy_node()
            if not self.port_handler.setBaudRate(baudrate):
                self.get_logger().error(
                    "Failed to set baud rate, shutting down node..."
                )
                self.destroy_node()
            self.get_logger().info("Serial communication succesful")
        except:
            self.get_logger().error("Failed to open port, shutting down node...")
            self.destroy_node()

    def callback_servo_command(self, msg):

        scs_comm_result, scs_error = self.packet_handler.WritePosEx(
            msg.servo_id, msg.pwm, SCS_MOVING_SPEED := 255, SCS_MOVING_ACC := 255
        )

        if scs_comm_result != scservo_def.COMM_SUCCESS:
            self.get_logger().error(
                f"Error in servo communication for servo id {msg.servo_id}: {scs_comm_result}"
            )

        # if scs_comm_result != scservo_def.COMM_SUCCESS:
        #     print("%s" % self.packet_handler.getTxRxResult(scs_comm_result))
        # elif scs_error != 0:
        #     print("%s" % self.packet_handler.getRxPacketError(scs_error))


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoDriverWaveshare()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
