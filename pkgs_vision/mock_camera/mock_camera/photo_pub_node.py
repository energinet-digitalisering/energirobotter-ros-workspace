import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class MockCameraNode(Node):

    def __init__(self):
        super().__init__("photo_pub_node")

        self.declare_parameter("frequency", 5.0)
        frequency = self.get_parameter("frequency").get_parameter_value().double_value

        self.declare_parameter("use_compressed", False)
        self.use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
        )

        # Publishers
        if self.use_compressed:
            self.publisher = self.create_publisher(CompressedImage, "/camera", 1)
        else:
            self.publisher = self.create_publisher(Image, "/camera", 1)

        # Timers
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)

        # Node variables
        img_path = "install/mock_camera/share/mock_camera/images/faces.jpg"
        self.image = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2RGB)

        self.cv_bridge = CvBridge()
        if self.use_compressed:
            self.msg_image = self.cv_bridge.cv2_to_compressed_imgmsg(self.image)
        else:
            self.msg_image = self.cv_bridge.cv2_to_imgmsg(self.image, encoding="rgb8")

        self.get_logger().info("Starting image publishing...")

    def timer_callback(self):
        self.publisher.publish(self.msg_image)


def main(args=None):
    rclpy.init(args=args)

    photo_pub_node = MockCameraNode()

    rclpy.spin(photo_pub_node)
    photo_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
