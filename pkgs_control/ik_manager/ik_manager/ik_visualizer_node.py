import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class IKVisualizerNode(Node):
    def __init__(self):
        super().__init__("ik_visualizer_node")

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Define the joint names and IK solution
        self.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
            "joint_8",
            "joint_9",
            "joint_10",
        ]
        self.ik_solution = [
            -0.07555402,
            -0.01767564,
            -0.17672099,
            -0.14084403,
            0.0177118,
            0.07346775,
            -0.01200479,
            0.07332419,
            0.13982251,
            0.1745678,
        ]  # Replace with your actual IK result

        # Publish the state periodically
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Add joint names and positions
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.ik_solution

        self.joint_state_pub.publish(joint_state_msg)
        self.get_logger().info("Published joint states to RViz")


def main(args=None):
    rclpy.init(args=args)
    node = IKVisualizerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
