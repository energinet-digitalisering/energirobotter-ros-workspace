import zmq
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Using String to send raw JSON data


class ReceiveTrackingNode(Node):
    def __init__(self):
        super().__init__("receive_tracking_node")

        # Parameters
        self.declare_parameter("ip_target", "127.0.0.1")
        ip_target = self.get_parameter("ip_target").get_parameter_value().string_value

        self.declare_parameter("socket", "5555")
        socket = self.get_parameter("socket").get_parameter_value().string_value

        # ZeroMQ setup
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect(f"tcp://{ip_target}:{socket}")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

        # ROS Publisher
        self.tracking_pub = self.create_publisher(String, "/tracking_data", 1)

        # Timer for polling ZeroMQ
        self.timer = self.create_timer(1.0 / 60, self.poll_zmq)  # 60Hz polling

    def poll_zmq(self):
        """Polls ZeroMQ for new messages and publishes to ROS."""
        if self.subscriber.poll(0):  # Check if data is available
            message = self.subscriber.recv_string()
            self.tracking_pub.publish(String(data=message))


def main(args=None):
    rclpy.init(args=args)
    node = ReceiveTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
