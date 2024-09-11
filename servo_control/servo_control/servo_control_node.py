import rclpy
from rclpy.node import Node
import std_msgs
import std_msgs.msg

from servo_control import servo_control


class ServoControlNode(Node):

    def __init__(self):
        super().__init__("servo_control_node")

        # Parameters

        self.declare_parameter("servo_id", 0)
        servo_id = self.get_parameter("servo_id").get_parameter_value().integer_value

        self.declare_parameter("operation_mode", "angle")
        operation_mode = (
            self.get_parameter("operation_mode").get_parameter_value().string_value
        )

        self.declare_parameter("com_protocol", "serial")
        com_protocol = (
            self.get_parameter("com_protocol").get_parameter_value().string_value
        )

        self.declare_parameter("com_port", "/dev/ttyACM0")
        com_port = self.get_parameter("com_port").get_parameter_value().string_value

        self.declare_parameter("control_frequency", 0.05)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        self.declare_parameter("pwm_min", 0)
        pwm_min = self.get_parameter("pwm_min").get_parameter_value().integer_value

        self.declare_parameter("pwm_max", 4095)
        pwm_max = self.get_parameter("pwm_max").get_parameter_value().integer_value

        self.declare_parameter("angle_min", 0)
        angle_min = self.get_parameter("angle_min").get_parameter_value().integer_value
        self.declare_parameter("angle_max", 180)
        angle_max = self.get_parameter("angle_max").get_parameter_value().integer_value

        self.declare_parameter("speed_max", 200.0)
        speed_max = self.get_parameter("speed_max").get_parameter_value().double_value

        self.declare_parameter("dir", 1)
        dir = self.get_parameter("dir").get_parameter_value().integer_value

        self.declare_parameter("gain_P", 1.0)
        gain_P = self.get_parameter("gain_P").get_parameter_value().double_value

        self.declare_parameter("gain_I", 0.0)
        gain_I = self.get_parameter("gain_I").get_parameter_value().double_value

        self.declare_parameter("gain_D", 0.0)
        gain_D = self.get_parameter("gain_D").get_parameter_value().double_value

        # Operation mode setup
        if operation_mode == "angle":
            # Subscriptions
            self.subscription = self.create_subscription(
                std_msgs.msg.Float64,
                "set_angle",
                self.callback_set_angle,
                1,
            )

            # Timers
            self.timer_angle = self.create_timer(
                self.control_frequency, self.callback_timer_angle
            )

        # Operation mode setup
        if operation_mode == "control":
            # Subscriptions
            self.subscription = self.create_subscription(
                std_msgs.msg.Float64,
                "set_error",
                self.callback_set_error,
                1,
            )

        # Node variables

        # Servo config
        self.servo = servo_control.ServoControl(
            pwm_min,
            pwm_max,
            angle_min,
            angle_max,
            speed_max,
            servo_id=servo_id,
            dir=dir,
            gain_P=gain_P,
            gain_I=gain_I,
            gain_D=gain_D,
            protocol=com_protocol,
            port=com_port,
        )

        if not self.servo.ready():
            self.get_logger().error(
                "Servo could not be initialised, shutting down node..."
            )
            self.destroy_node()

        self.desired_angle = self.servo.angle_init

    def callback_set_angle(self, msg):
        self.desired_angle = msg.data

    def callback_timer_angle(self):
        self.servo.reach_angle(self.control_frequency, self.desired_angle)

    def callback_set_error(self, msg):
        error = msg.data
        self.servo.compute_control(self.control_frequency, error)


def main(args=None):
    rclpy.init(args=args)

    servo_control_node = ServoControlNode()

    rclpy.spin(servo_control_node)
    servo_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
