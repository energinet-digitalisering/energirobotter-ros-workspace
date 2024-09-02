import rclpy
from rclpy.node import Node
import vision_msgs.msg

from servo_control import servo_control


class FaceFollowingNode(Node):

    def __init__(self):
        super().__init__("face_following_node")

        # Parameters
        self.declare_parameter("timer_period", 0.05)
        self.timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )

        self.declare_parameter("image_w", 1280)
        self.image_w = self.get_parameter("image_w").get_parameter_value().integer_value

        self.declare_parameter("image_h", 720)
        self.image_h = self.get_parameter("image_h").get_parameter_value().integer_value

        self.declare_parameter("fov_w", 70)
        self.fov_w = self.get_parameter("fov_w").get_parameter_value().integer_value

        self.declare_parameter("fov_h", 50)
        self.fov_h = self.get_parameter("fov_h").get_parameter_value().integer_value

        self.declare_parameter("servo_pos_min", 0)
        servo_pos_min = (
            self.get_parameter("servo_pos_min").get_parameter_value().integer_value
        )
        self.declare_parameter("servo_pos_max", 180)
        servo_pos_max = (
            self.get_parameter("servo_pos_max").get_parameter_value().integer_value
        )

        self.declare_parameter("servo_speed_max", 100.0)
        servo_speed_max = (
            self.get_parameter("servo_speed_max").get_parameter_value().double_value
        )

        self.declare_parameter("servo_pan_dir", 1)
        servo_pan_dir = (
            self.get_parameter("servo_pan_dir").get_parameter_value().integer_value
        )

        self.declare_parameter("servo_tilt_dir", 1)
        servo_tilt_dir = (
            self.get_parameter("servo_tilt_dir").get_parameter_value().integer_value
        )

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

        # Subscriptions
        self.subscription = self.create_subscription(
            vision_msgs.msg.BoundingBox2D,
            "/face_bounding_box",
            self.bounding_box_callback,
            1,
        )

        # Timers
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Node variables
        self.detection_time_stamp = self.get_clock().now()
        self.detection_time_stop_error = 0.5  # Seconds
        self.detection_time_stop_follow = 4.0  # Seconds

        self.center_x = round(self.image_w / 2)
        self.center_y = round(self.image_h / 2)

        self.target_x = self.center_x
        self.target_y = self.center_y

        # Servo config
        self.servo_pan = servo_control.ServoControl(
            servo_pos_min,
            servo_pos_max,
            servo_speed_max,
            servo_pan_dir,
            servo_gain_P,
            servo_gain_I,
            servo_gain_D,
        )

        # self.servo_tilt = servo_control.ServoControl(
        #     servo_pos_min,
        #     servo_pos_max,
        #     servo_speed_max,
        #     servo_tilt_dir,
        #     servo_gain_P,
        #     servo_gain_I,
        #     servo_gain_D,
        # )

    def bounding_box_callback(self, msg):

        self.target_x = msg.center.position.x
        self.target_y = msg.center.position.y

        self.detection_time_stamp = self.get_clock().now()

    def timer_callback(self):

        time_current = self.get_clock().now()
        time_diff = (
            time_current - self.detection_time_stamp
        ).nanoseconds / 1000000000  # Seconds

        # Stop following if detection is gone for too long
        if time_diff > self.detection_time_stop_follow:
            self.servo_pan.reset_position(self.timer_period)
            # self.servo_tilt.reset_position(self.timer_period)
            return

        # Reset error if detection is gone for too long
        if time_diff > self.detection_time_stop_error:
            error_pan = 0.0
            error_tilt = 0.0

        else:
            error_pan = self.center_x - self.target_x
            error_tilt = self.center_y - self.target_y

        # Send command to servos
        self.servo_pan.compute_control(error_pan, self.timer_period)
        # self.servo_tilt.compute_control(error_tilt, self.timer_period)


def main(args=None):
    rclpy.init(args=args)

    face_following_node = FaceFollowingNode()

    rclpy.spin(face_following_node)
    face_following_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
