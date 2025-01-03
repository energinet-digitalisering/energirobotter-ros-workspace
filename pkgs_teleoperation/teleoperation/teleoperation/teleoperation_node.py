from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from teleoperation.src.vuer_app import VuerApp
from teleoperation.src.vuer_transformer import VuerTransformer


class TeleoperationNode(Node):

    def __init__(self):
        super().__init__("teleoperation_node")

        # Parameters
        self.declare_parameter("fps", 30)
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value

        # Subscribers
        self.subscription_image_left = self.create_subscription(
            CompressedImage, "/image_left", self.callback_image_left, 1
        )

        self.subscription_image_right = self.create_subscription(
            CompressedImage, "/image_right", self.callback_image_right, 1
        )

        # Timers
        self.timer = self.create_timer(1.0 / self.fps, self.callback_timer)

        # Variables
        self.image_left = None
        self.image_right = None

        self.cv_bridge = CvBridge()

        self.vuer_app = VuerApp()
        self.vuer_transformer = VuerTransformer()

    def callback_image_left(self, msg):
        self.image_left = self.cv_bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding="rgb8"
        )

    def callback_image_right(self, msg):
        self.image_right = self.cv_bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding="rgb8"
        )

    def callback_timer(self):
        self.vuer_app.update_frames(self.image_left, self.image_right)
        head_mat, left_wrist_mat, right_wrist_mat = self.vuer_transformer.process(
            self.vuer_app
        )



def main(args=None):
    rclpy.init(args=args)

    teleoperation_node = TeleoperationNode()

    rclpy.spin(teleoperation_node)
    teleoperation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
