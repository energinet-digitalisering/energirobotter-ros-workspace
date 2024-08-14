import os

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MockCamera(Node):

    def __init__(self):
        super().__init__("mock_camera")

        self.declare_parameter("timer_period", 0.5)
        timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )

        # Publishers
        self.publisher_ = self.create_publisher(Image, "camera", 10)

        # Timers
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Node variables
        img_path = "install/mock_camera/share/mock_camera/images/faces.jpg"

        self.cv_bridge = CvBridge()
        self.image = cv2.imread(img_path)

        self.get_logger().info("Starting image publishing...")

    def timer_callback(self):
        self.publisher_.publish(
            self.cv_bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        )


def main(args=None):
    rclpy.init(args=args)

    mock_camera = MockCamera()

    rclpy.spin(mock_camera)
    mock_camera.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
