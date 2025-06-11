import numpy as np
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CommandTestNode(Node):

    def __init__(self):
        super().__init__("command_test_node")

        # Parameters
        self.declare_parameter("angle_left", 0.0)
        self.angle_left = (
            self.get_parameter("angle_left").get_parameter_value().double_value
        )

        self.declare_parameter("angle_right", 0.0)
        self.angle_right = (
            self.get_parameter("angle_right").get_parameter_value().double_value
        )

        # Publishers
        joint_state_pub = self.create_publisher(JointState, "/joint_states", 1)

        # Prepare message
        joint_state_msg = JointState()
        joint_state_msg.name = [
            "joint_left_shoulder_pitch",
            "joint_right_shoulder_pitch",
        ]
        joint_positions = [
            np.deg2rad(self.angle_left),
            np.deg2rad(self.angle_right),
        ]
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.position = joint_positions

        # Publish once
        self.get_logger().info("Published joint state command.")
        joint_state_pub.publish(joint_state_msg)

        # Sleep to ensure message is sent
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    node_handle = CommandTestNode()
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
