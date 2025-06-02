import rclpy
from rclpy.node import Node

from webrtc_server_camera.src.webrtc_server_camera import WebRTCServerCamera


class WebRTCServerCameraNode(Node):

    def __init__(self):
        super().__init__("webrtc_server_camera_node")

        webrtc_server_camera = WebRTCServerCamera()


def main(args=None):
    rclpy.init(args=args)

    webrtc_server_camera_node = WebRTCServerCameraNode()

    rclpy.spin(webrtc_server_camera_node)
    webrtc_server_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
