"""
Managing ROS communication for all servos in Elrik
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from servo_control.src.elrik_driver_arms import ElrikDriverArms
from servo_control.src.elrik_driver_hands import ElrikDriverHands


class ServoManagerNode(Node):

    def __init__(self):
        super().__init__("servo_manager_node")

        # Parameters
        self.declare_parameter("control_frequency", 0.5)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        # Subscriptions
        self.sub_joints_arms = self.create_subscription(
            JointState, "/joint_states", self.callback_joints_arms, 1
        )
        self.sub_joints_hands = self.create_subscription(
            JointState, "/joint_states_hands", self.callback_joints_hands, 1
        )

        # Publishers

        # Timers
        self.timer = self.create_timer(self.control_frequency, self.callback_timer)

        # Node variables
        config_folder_path = "install/elrik_bringup/share/elrik_bringup/config/servos"

        # Configure arm servo manager
        json_files_arms = [
            f"{config_folder_path}/servo_arm_left_params.json",
            f"{config_folder_path}/servo_arm_right_params.json",
        ]
        self.driver_arms = ElrikDriverArms(json_files_arms, self.control_frequency)
        self.servo_commands_arms = self.driver_arms.get_default_servo_commands()

        # Configure hands servo manager
        json_files_hands = [
            f"{config_folder_path}/servo_hand_left_params.json",
            f"{config_folder_path}/servo_hand_right_params.json",
        ]
        self.driver_hands = ElrikDriverHands(json_files_hands, self.control_frequency)
        self.servo_commands_hands = self.driver_hands.get_default_servo_commands()

    def callback_joints_arms(self, msg):
        self.servo_commands_arms = dict(zip(msg.name, np.rad2deg(msg.position)))

        # self.get_logger().info(f"Updated joint positions: {joint_positions}")

    def callback_joints_hands(self, msg):
        self.servo_commands_hands = dict(zip(msg.name, np.rad2deg(msg.position)))

    def callback_timer(self):
        self.driver_arms.update_feedback()
        self.driver_arms.command_servos(self.servo_commands_arms)

        self.driver_hands.update_feedback()
        self.driver_hands.command_servos(self.servo_commands_hands)


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoManagerNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
