import numpy as np

import rclpy
from rclpy.node import Node
import vision_msgs.msg


class FaceFollowingNode(Node):

    def __init__(self):
        super().__init__("face_following_node")

        # Parameters
        self.declare_parameter("timer_period", 0.5)
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

        # Timers
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Node variables
        self.center_x = round(self.image_w / 2)
        self.center_y = round(self.image_h / 2)

        self.target_x = self.center_x
        self.target_y = self.center_y

        # Servo config
        self.servo_speed_min = -1.0
        self.servo_speed_max = 1.0
        self.servo_p_gain = 1.0

        self.servo_pan_dir = 1  # Direction config for upside-down placement

        self.servo_tilt_dir = 1  # Direction config for upside-down placement

    def bounding_box_callback(self, msg):

        self.target_x = msg.center.position.x
        self.target_y = msg.center.position.y

    def timer_callback(self):
        # Compute control
        pan_control = self.servo_p_gain * (self.center_x - self.target_x)
        tilt_control = self.servo_p_gain * (self.center_y - self.target_y)

        # Clamp values between min and max speed
        pan_control, tilt_control = np.clip(
            [pan_control, tilt_control], self.servo_speed_min, self.servo_speed_max
        )

        # TODO: Add control to current position of servos - how to get that - encoders or dead reckoning?


def main(args=None):
    rclpy.init(args=args)

    face_following_node = FaceFollowingNode()

    rclpy.spin(face_following_node)
    face_following_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
