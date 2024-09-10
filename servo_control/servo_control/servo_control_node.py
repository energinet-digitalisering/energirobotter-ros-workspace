import rclpy
from rclpy.node import Node
import std_msgs
import std_msgs.msg

from servo_control import servo_control


class ServoControlNode(Node):

    def __init__(self):
        super().__init__("servo_control_node")

        # Parameters
        self.declare_parameter("timer_period", 0.05)
        self.timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )

        self.declare_parameter("operation_mode_angle", False)
        self.operation_mode_angle = (
            self.get_parameter("operation_mode_angle").get_parameter_value().bool_value
        )

        self.declare_parameter("operation_mode_control", False)
        self.operation_mode_control = (
            self.get_parameter("operation_mode_control")
            .get_parameter_value()
            .bool_value
        )

        self.declare_parameter("servo_id", 0)
        servo_id = self.get_parameter("servo_id").get_parameter_value().integer_value

        self.declare_parameter("servo_pwm_min", 0)
        servo_pwm_min = (
            self.get_parameter("servo_pwm_min").get_parameter_value().integer_value
        )

        self.declare_parameter("servo_pwm_max", 4095)
        servo_pwm_max = (
            self.get_parameter("servo_pwm_max").get_parameter_value().integer_value
        )

        self.declare_parameter("servo_angle_min", 0)
        servo_angle_min = (
            self.get_parameter("servo_angle_min").get_parameter_value().integer_value
        )
        self.declare_parameter("servo_angle_max", 180)
        servo_angle_max = (
            self.get_parameter("servo_angle_max").get_parameter_value().integer_value
        )

        self.declare_parameter("servo_speed_max", 100.0)
        servo_speed_max = (
            self.get_parameter("servo_speed_max").get_parameter_value().double_value
        )

        self.declare_parameter("servo_dir", 1)
        servo_dir = self.get_parameter("servo_dir").get_parameter_value().integer_value

        self.declare_parameter("servo_gain_P", 1.0)
        servo_gain_P = (
            self.get_parameter("servo_gain_P").get_parameter_value().double_value
        )

        self.declare_parameter("servo_gain_I", 0.0)
        servo_gain_I = (
            self.get_parameter("servo_gain_I").get_parameter_value().double_value
        )

        self.declare_parameter("servo_gain_D", 0.0)
        servo_gain_D = (
            self.get_parameter("servo_gain_D").get_parameter_value().double_value
        )

        # Checks
        if not self.operation_mode_angle and not self.operation_mode_control:
            self.get_logger().info("No operation mode set.")
            exit()
        if self.operation_mode_angle and self.operation_mode_control:
            self.get_logger().info("Multiple operation modes set, please only set one.")
            exit()

        if self.operation_mode_angle:
            # Subscriptions
            self.subscription = self.create_subscription(
                std_msgs.msg.Float64,
                "servo_" + str(servo_id) + "/set_angle",
                self.callback_set_angle,
                1,
            )

            # Timers
            self.timer_angle = self.create_timer(
                self.timer_period, self.callback_timer_angle
            )

        # Node variables

        # Servo config
        self.servo = servo_control.ServoControl(
            servo_pwm_min,
            servo_pwm_max,
            servo_angle_min,
            servo_angle_max,
            servo_speed_max,
            dir=servo_dir,
            gain_P=servo_gain_P,
            gain_I=servo_gain_I,
            gain_D=servo_gain_D,
            protocol="i2c",
        )

        self.desired_angle = self.servo.angle_init

    def callback_set_angle(self, msg):
        self.desired_angle = msg.data

    def callback_timer_angle(self):
        self.servo.reach_angle(self.desired_angle)


def main(args=None):
    rclpy.init(args=args)

    servo_control_node = ServoControlNode()

    rclpy.spin(servo_control_node)
    servo_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
