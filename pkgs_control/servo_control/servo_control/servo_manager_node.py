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
        self.declare_parameter("control_frequency_arms", 0.1)
        self.control_frequency_arms = (
            self.get_parameter("control_frequency_arms")
            .get_parameter_value()
            .double_value
        )

        self.declare_parameter("control_frequency_hands", 0.01)
        self.control_frequency_hands = (
            self.get_parameter("control_frequency_hands")
            .get_parameter_value()
            .double_value
        )

        # Subscriptions
        self.sub_joints_arms = self.create_subscription(
            JointState, "/joint_states", self.callback_joints_arms, 1
        )
        self.sub_joints_hands = self.create_subscription(
            JointState, "/joint_states_hands", self.callback_joints_hands, 1
        )

        # Publishers

        # DEBUG
        self.pub_speeds = self.create_publisher(JointState, "/log_speeds", 10)
        # DEBUG END

        # Timers
        self.timer_arms = self.create_timer(
            self.control_frequency_arms, self.callback_timer_arms
        )

        self.timer_hands = self.create_timer(
            self.control_frequency_hands, self.callback_timer_hands
        )

        # Node variables
        config_folder_path = "install/elrik_bringup/share/elrik_bringup/config/servos"

        # Configure arm servo manager
        json_files_arms = [
            f"{config_folder_path}/servo_arm_left_params.json",
            # f"{config_folder_path}/servo_arm_right_params.json",
        ]
        self.driver_arms = ElrikDriverArms(
            json_files_arms, self.control_frequency_arms, synchronise_speed=True
        )
        self.servo_commands_arms = self.driver_arms.get_default_servo_commands()

        # Configure hands servo manager
        json_files_hands = [
            # f"{config_folder_path}/servo_hand_left_params.json",
            f"{config_folder_path}/servo_hand_right_params.json",
            # f"{config_folder_path}/servo_test.json",
        ]
        self.driver_hands = ElrikDriverHands(
            json_files_hands, self.control_frequency_hands, synchronise_speed=False
        )
        self.servo_commands_hands = self.driver_hands.get_default_servo_commands()

    def callback_joints_arms(self, msg):
        self.servo_commands_arms = dict(zip(msg.name, np.rad2deg(msg.position)))

    def callback_joints_hands(self, msg):
        self.servo_commands_hands = dict(zip(msg.name, np.rad2deg(msg.position)))

        for servo_name, servo in self.driver_hands.servos.items():
            command = self.servo_commands_hands[servo_name]
            angle_mapped = self.driver_hands.map_finger_to_servo(servo, command)
            self.servo_commands_hands[servo_name] = angle_mapped

    def callback_timer_arms(self):
        self.driver_arms.update_feedback()
        self.driver_arms.command_servos(self.servo_commands_arms)

        # DEBUG
        positions = self.driver_arms.get_servo_angles()
        speeds = self.driver_arms._compute_relative_speeds(self.servo_commands_arms)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(speeds.keys())
        msg.position = list(positions.values())
        msg.velocity = list(speeds.values())

        self.pub_speeds.publish(msg)
        # DEBUG END

    def callback_timer_hands(self):
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
