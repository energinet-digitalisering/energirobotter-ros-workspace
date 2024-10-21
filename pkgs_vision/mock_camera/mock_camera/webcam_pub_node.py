import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MockCameraNode(Node):

    def __init__(self):
        super().__init__("webcam_pub_node")

        self.declare_parameter("timer_period", 0.01)
        timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )

        # Publishers
        self.publisher = self.create_publisher(Image, "/camera", 1)

        # Timers
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Node variables
        self.cv_bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)

        self.get_logger().info("Starting image publishing...")

    def timer_callback(self):

        _, frame = self.capture.read()
        self.image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        self.publisher.publish(
            self.cv_bridge.cv2_to_imgmsg(self.image, encoding="rgb8")
        )


def main(args=None):
    rclpy.init(args=args)

    webcam_pub_node = MockCameraNode()

    rclpy.spin(webcam_pub_node)
    webcam_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
