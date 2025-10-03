"""
Managing ROS communication for all servos in Wattson
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from servo_control.src.driver_waveshare import DriverWaveshare


class ServoManagerNode(Node):

    def __init__(self):
        super().__init__("servo_manager_node")

        # Parameters
        self.declare_parameter("control_frequency", 10.0)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        self.declare_parameter("finger_mapping_enabled", True)
        self.finger_mapping_enabled = (
            self.get_parameter("finger_mapping_enabled")
            .get_parameter_value()
            .bool_value
        )

        self.declare_parameter(
            "config_folder_path",
            "install/energirobotter_bringup/share/energirobotter_bringup/config/servos",
        )
        config_folder_path = (
            self.get_parameter("config_folder_path").get_parameter_value().string_value
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
            1.0 / self.control_frequency, self.callback_timer_arms
        )

        self.timer_hands = self.create_timer(
            1.0 / (1.0 * self.control_frequency), self.callback_timer_hands
        )

        # Configure servo managers
        # Arms
        json_files_arms_left = [
            f"{config_folder_path}/servo_arm_left_params.json",
            f"{config_folder_path}/servo_head_params.json",
        ]
        self.servo_driver_arms_left = DriverWaveshare(
            json_files_arms_left,
            self.control_frequency,
            port_path="/dev/ttyUSB0",
            baudrate=921600,
        )
        self.servo_driver_arms_left.initialize()

        json_files_arms_right = [
            f"{config_folder_path}/servo_arm_right_params.json",
        ]
        self.servo_driver_arms_right = DriverWaveshare(
            json_files_arms_right,
            self.control_frequency,
            port_path="/dev/ttyUSB1",
            baudrate=921600,
        )
        self.servo_driver_arms_right.initialize()

        # Hands
        json_files_hands = [
            f"{config_folder_path}/servo_hand_left_params.json",
            f"{config_folder_path}/servo_hand_right_params.json",
        ]
        self.servo_driver_hands = DriverWaveshare(
            json_files_hands,
            self.control_frequency,
            port_path="/dev/ttyUSB2",
            baudrate=921600,
        )
        self.servo_driver_hands.initialize()

        # Node variables
        self.servo_commands_arms = {}
        self.servo_commands_hands = {}

    def callback_joints_arms(self, msg):
        self.servo_commands_arms = dict(zip(msg.name, np.rad2deg(msg.position)))

    def callback_joints_hands(self, msg):
        self.servo_commands_hands = dict(zip(msg.name, np.rad2deg(msg.position)))

        # Map angles to servo range for fingers
        if self.finger_mapping_enabled:
            for servo_name in self.servo_commands_hands:
                if servo_name not in self.servo_driver_hands.servos:
                    continue

                servo = self.servo_driver_hands.servos[servo_name]
                command = self.servo_commands_hands[servo_name]

                angle_mapped = self.servo_driver_hands.map_finger_to_servo(
                    servo, command
                )
                self.servo_commands_hands[servo_name] = angle_mapped

    def callback_timer_arms(self):
        if not self.servo_commands_arms:
            self.get_logger().info(f"No arm commands received yet...", once=True)
        else:
            self.get_logger().info(f"Arm commands received!", once=True)

        # Update servos
        self.servo_driver_arms_left.update_feedback()
        self.servo_driver_arms_left.command_servos(self.servo_commands_arms)

        self.servo_driver_arms_right.update_feedback()
        self.servo_driver_arms_right.command_servos(self.servo_commands_arms)

        # # DEBUG
        # temperatures = self.servo_driver_arms.get_servo_temperatures()
        # # positions = self.servo_driver_arms.get_servo_angles()

        # msg = JointState()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.name = list(temperatures.keys())
        # msg.position = list(temperatures.values())

        # self.pub_speeds.publish(msg)
        # # DEBUG END

    def callback_timer_hands(self):
        if not self.servo_commands_hands:
            self.get_logger().info(f"No hand commands received yet...", once=True)
        else:
            self.get_logger().info(f"Hand commands received!", once=True)

        # Update servos
        self.servo_driver_hands.update_feedback()
        self.servo_driver_hands.command_servos(self.servo_commands_hands)


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoManagerNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
