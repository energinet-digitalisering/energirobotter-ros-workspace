import zmq
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Using String to send raw JSON data


class ReceiveTrackingNode(Node):
    def __init__(self):
        super().__init__("receive_tracking_node")

        # Parameters
        self.declare_parameter("ip_target", "127.0.0.1")
        self.ip_target = (
            self.get_parameter("ip_target").get_parameter_value().string_value
        )

        self.declare_parameter("port", 5557)
        self.port = self.get_parameter("port").get_parameter_value().integer_value

        # ZeroMQ setup
        self.subscriber = zmq.Context().socket(zmq.SUB)
        self.subscriber.connect(f"tcp://{self.ip_target}:{self.port}")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

        # ROS Publisher
        self.tracking_pub = self.create_publisher(String, "/tracking_data", 1)

        # Timer for polling ZeroMQ
        self.timer = self.create_timer(1.0 / 50.0, self.timer_poll_zmq)  # 50Hz polling

    def timer_poll_zmq(self):
        """Polls ZeroMQ for new messages and publishes to ROS."""
        self.get_logger().info(f"Polling {self.ip_target}:{self.port}...", once=True)

        if self.subscriber.poll(0):  # Check if data is available
            self.get_logger().info(f"Message received", once=True)

            tracking_data = self.subscriber.recv_string()
            self.tracking_pub.publish(String(data=tracking_data))


def main(args=None):
    rclpy.init(args=args)
    node = ReceiveTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
