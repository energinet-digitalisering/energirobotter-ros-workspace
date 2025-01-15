"""
Managing ROS communication for all servos in Elrik
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from servo_control.src.elrik_driver_arms import ElrikDriverArms


class ServoManagerNode(Node):

    def __init__(self):
        super().__init__("servo_manager_node")

        # Parameters
        self.declare_parameter("control_frequency", 0.5)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        # Subscriptions
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.callback_joint_states, 1
        )

        # Publishers

        # Timers
        self.timer = self.create_timer(self.control_frequency, self.callback_timer)

        # Node variables
        config_folder_path = "install/elrik_bringup/share/elrik_bringup/config/servos"

        self.driver_arms = ElrikDriverArms(config_folder_path, self.control_frequency)

        self.servo_commands = self.driver_arms.get_default_servo_commands()

    def callback_joint_states(self, msg):
        self.servo_commands = dict(zip(msg.name, np.rad2deg(msg.position)))

        # self.get_logger().info(f"Updated joint positions: {joint_positions}")

    def callback_timer(self):
        self.driver_arms.update_feedback()
        self.driver_arms.command_servos(self.servo_commands)


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoManagerNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
