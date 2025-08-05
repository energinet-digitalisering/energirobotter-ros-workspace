import asyncio
import threading

import rclpy
from rclpy.node import Node

from webrtc_client_camera.src.webrtc_client_camera import WebRTCClientCamera


class WebRTCClientCameraNode(Node):
    def __init__(self):
        super().__init__("webrtc_client_camera_node")

        # Parameters
        self.declare_parameter("stereo_enabled", False)
        stereo_enabled = (
            self.get_parameter("stereo_enabled").get_parameter_value().bool_value
        )

        # Create server
        self.client = WebRTCClientCamera(stereo_enabled=stereo_enabled)

        # Run the async WebRTC server in its own event loop in a thread
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        self.get_logger().info("Starting WebRTC client...")

        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.client.connect())
        self.loop.run_forever()

    def destroy_node(self):
        self.get_logger().info("Shutting down WebRTC client...")

        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.client.stop(), self.loop)
            self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join()

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    webrtc_client_camera_node = WebRTCClientCameraNode()

    rclpy.spin(webrtc_client_camera_node)
    webrtc_client_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
