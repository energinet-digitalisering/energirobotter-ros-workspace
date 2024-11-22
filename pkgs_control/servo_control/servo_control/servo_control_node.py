import rclpy
from rclpy.node import Node
import std_msgs.msg

from energirobotter_interfaces.msg import ServoCommand
from servo_control.src import servo_control


class ServoControlNode(Node):

    def __init__(self):
        super().__init__("servo_control_node")

        # Parameters

        self.declare_parameter("servo_id", 0)
        self.servo_id = (
            self.get_parameter("servo_id").get_parameter_value().integer_value
        )

        self.declare_parameter("operation_mode", "angle")
        operation_mode = (
            self.get_parameter("operation_mode").get_parameter_value().string_value
        )

        self.declare_parameter("driver_device", "waveshare")
        driver_device = (
            self.get_parameter("driver_device").get_parameter_value().string_value
        )

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

        self.declare_parameter("angle_software_min", 0)
        angle_software_min = (
            self.get_parameter("angle_software_min").get_parameter_value().integer_value
        )

        self.declare_parameter("angle_max", 180)
        angle_max = self.get_parameter("angle_max").get_parameter_value().integer_value

        self.declare_parameter("angle_software_max", 180)
        angle_software_max = (
            self.get_parameter("angle_software_max").get_parameter_value().integer_value
        )

        self.declare_parameter("speed_max", 200)
        speed_max = self.get_parameter("speed_max").get_parameter_value().integer_value

        self.declare_parameter("dir", 1)
        dir = self.get_parameter("dir").get_parameter_value().integer_value

        self.declare_parameter("gain_P", 1.0)
        gain_P = self.get_parameter("gain_P").get_parameter_value().double_value

        self.declare_parameter("gain_I", 0.0)
        gain_I = self.get_parameter("gain_I").get_parameter_value().double_value

        self.declare_parameter("gain_D", 0.0)
        gain_D = self.get_parameter("gain_D").get_parameter_value().double_value

        # Publishers
        self.publisher = self.create_publisher(
            ServoCommand, "/" + driver_device + "/servo_command", 1
        )

        # Subscriptions
        if driver_device == "waveshare":
            self.subscription = self.create_subscription(
                ServoCommand,
                "waveshare/servo_feedback",
                self.callback_servo_feedback,
                1,
            )

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
                self.control_frequency, self.callback_timer_set_angle
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

        feedback_enabled = True if driver_device == "waveshare" else False

        self.get_logger().info(f"Initialising servo with id {self.servo_id}...")

        # Servo config
        self.servo = servo_control.ServoControl(
            pwm_min,
            pwm_max,
            angle_min,
            angle_software_min,
            angle_max,
            angle_software_max,
            speed_max,
            dir=dir,
            gain_P=gain_P,
            gain_I=gain_I,
            gain_D=gain_D,
            feedback_enabled=feedback_enabled,
        )

        # Node variables
        self.desired_angle = self.servo.angle_init

    def publish(self, angle, pwm):

        msg = ServoCommand()
        msg.servo_id = self.servo_id
        msg.angle = angle
        msg.pwm = pwm

        self.publisher.publish(msg)

    def callback_servo_command(self, msg):
        if msg.servo_id != self.servo_id:
            return

        self.servo.set_feedback_pwm(msg.pwm)

    def callback_set_angle(self, msg):
        self.desired_angle = msg.data

    def callback_timer_set_angle(self):
        angle, pwm = self.servo.reach_angle(self.control_frequency, self.desired_angle)
        self.publish(angle, pwm)

    def callback_set_error(self, msg):
        error = msg.data
        angle, pwm = self.servo.compute_control(self.control_frequency, error)
        self.publish(angle, pwm)


def main(args=None):
    rclpy.init(args=args)

    servo_control_node = ServoControlNode()

    rclpy.spin(servo_control_node)
    servo_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
