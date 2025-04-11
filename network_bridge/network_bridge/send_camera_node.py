import zmq
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage


class SendCameraNode(Node):
    def __init__(self):
        super().__init__("send_camera_node")

        # Parameters
        self.declare_parameter("ip_target", "*")
        ip_target = self.get_parameter("ip_target").get_parameter_value().string_value

        self.declare_parameter("port", 5555)
        self.port = self.get_parameter("port").get_parameter_value().integer_value

        self.declare_parameter("use_compressed", True)
        self.use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
        )

        # ROS Subscription
        if self.use_compressed:
            self.subscription = self.create_subscription(
                CompressedImage, "/camera", self.camera_callback, 10
            )
        else:
            self.subscription = self.create_subscription(
                Image, "/camera", self.camera_callback, 10
            )

        # ZeroMQ Publisher
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{ip_target}:{self.port}")

        # Node variables
        self.cv_bridge = CvBridge()

    def camera_callback(self, msg):
        if self.use_compressed:
            image = self.cv_bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="rgb8"
            )
        else:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Convert to JPEG to reduce data size
        _, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])

        # Send the image via ZeroMQ
        self.socket.send(buffer.tobytes())


def main(args=None):
    rclpy.init(args=args)
    send_camera_node = SendCameraNode()
    rclpy.spin(send_camera_node)
    send_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
