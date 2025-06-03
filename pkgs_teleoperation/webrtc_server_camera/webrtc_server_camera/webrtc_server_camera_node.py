import asyncio
import threading

import rclpy
from rclpy.node import Node

from webrtc_server_camera.src.webrtc_server_camera import WebRTCServerCamera


class WebRTCServerCameraNode(Node):
    def __init__(self):
        super().__init__("webrtc_server_camera_node")

        self.server = WebRTCServerCamera()

        # Run the async WebRTC server in its own event loop in a thread
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.server.start())
        self.loop.run_forever()

    def destroy_node(self):
        self.get_logger().info("Shutting down WebRTC server...")

        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.server.stop(), self.loop)
            self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join()

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    webrtc_server_camera_node = WebRTCServerCameraNode()

    rclpy.spin(webrtc_server_camera_node)
    webrtc_server_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
