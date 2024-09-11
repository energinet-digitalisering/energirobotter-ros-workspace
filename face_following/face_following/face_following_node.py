import rclpy
from rclpy.node import Node
import std_msgs
import std_msgs.msg
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

        # Subscriptions
        self.subscription = self.create_subscription(
            vision_msgs.msg.BoundingBox2D,
            "/face_bounding_box",
            self.bounding_box_callback,
            1,
        )

        # Publishers
        self.publisher_servo_pan = self.create_publisher(
            std_msgs.msg.Float64, "/servo_pan/set_error", 1
        )
        self.publisher_servo_tilt = self.create_publisher(
            std_msgs.msg.Float64, "/servo_tilt/set_error", 1
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
            # self.servo_pan.reset_position(self.timer_period)
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

        self.publisher_servo_pan.publish(std_msgs.msg.Float64(data=float(error_pan)))
        self.publisher_servo_tilt.publish(std_msgs.msg.Float64(data=float(error_tilt)))


def main(args=None):
    rclpy.init(args=args)

    face_following_node = FaceFollowingNode()

    rclpy.spin(face_following_node)
    face_following_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
