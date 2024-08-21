import numpy as np

import rclpy
from rclpy.node import Node
import vision_msgs.msg


class FaceFollowingNode(Node):

    def __init__(self):
        super().__init__("face_following_node")

        # Parameters
        self.declare_parameter("timer_period", 0.1)
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

        self.declare_parameter("servo_speed_min", -100.0)
        servo_speed_min = (
            self.get_parameter("servo_speed_min").get_parameter_value().double_value
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

        self.declare_parameter("servo_p_gain", 1.0)
        servo_p_gain = (
            self.get_parameter("servo_p_gain").get_parameter_value().double_value
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
        self.center_x = round(self.image_w / 2)
        self.center_y = round(self.image_h / 2)

        self.target_x = self.center_x
        self.target_y = self.center_y

        # Servo config
        self.servo_pos_min = 0  # PWM
        self.servo_pos_max = 1000  # PWM
        self.servo_speed_min = -10.0  # PWM/s
        self.servo_speed_max = 10.0  # PWM/s
        self.servo_p_gain = 1.0

        # Direction config for upside-down placement (-1 or 1)
        self.servo_pan_dir = 1
        self.servo_tilt_dir = 1

        self.servo_pan_pos = np.round(self.servo_pos_max / 2)
        self.servo_tilt_pos = np.round(self.servo_pos_max / 2)

        # TODO: Send initial pos to servos

    def bounding_box_callback(self, msg):

        self.target_x = msg.center.position.x
        self.target_y = msg.center.position.y

    def timer_callback(self):
        # Compute control
        pan_vel_control = (
            self.servo_p_gain * (self.center_x - self.target_x) * self.timer_period
        )
        tilt_vel_control = (
            self.servo_p_gain * (self.center_y - self.target_y) * self.timer_period
        )

        # Clamp values between min and max speed
        pan_vel_control, tilt_vel_control = np.clip(
            [pan_vel_control, tilt_vel_control],
            self.servo_speed_min * self.timer_period,
            self.servo_speed_max * self.timer_period,
        )

        self.servo_pan_pos += self.servo_pan_dir * pan_vel_control
        self.servo_tilt_pos += self.servo_tilt_dir * tilt_vel_control

        # Clamp values between min and max position
        self.servo_pan_pos, self.servo_tilt_pos = np.clip(
            [self.servo_pan_pos, self.servo_tilt_pos],
            self.servo_pos_min,
            self.servo_pos_max,
        )

        # TODO: Send pos to servos


def main(args=None):
    rclpy.init(args=args)

    face_following_node = FaceFollowingNode()

    rclpy.spin(face_following_node)
    face_following_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
