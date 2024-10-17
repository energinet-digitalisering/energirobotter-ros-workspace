from typing import Optional

import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
import std_msgs

from animation_player.src import csv_reader


class AnimationPlayerNode(Node):

    def __init__(self):
        super().__init__("animation_player_node")

        # Parameters

        self.declare_parameter("fps", 24)
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value

        self.declare_parameter("csv_file_path", "")
        self.csv_file_path = (
            self.get_parameter("csv_file_path").get_parameter_value().string_value
        )

        # CSV file setup
        self.csv_reader = csv_reader.CSVReader(self.csv_file_path)

        # Lifecycle Node timers and publishers
        self.timer: Optional[Timer] = None

        self.joint_idx: int = {}
        self.joint_publishers: Optional[Publisher] = {}
        for idx, joint in enumerate(self.csv_reader.get_header()):
            self.joint_idx[joint] = idx
            self.joint_publishers[joint] = None

        # Node variables

    def __del__(self):
        self.csv_reader.close()

    ##################### Functions #####################

    ##################### Callbacks #####################

    def callback_timer(self):

        if None in self.joint_publishers.values():
            return

        row = self.csv_reader.get_next_row()

        for joint in self.joint_publishers.keys():
            data = row[self.joint_idx[joint]]
            self.joint_publishers[joint].publish(std_msgs.msg.Float64(data=float(data)))

    ##################### Lifecyle Node Functions #####################

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        # Timers
        self.timer = self.create_timer(1.0 / self.fps, self.callback_timer)

        # Publishers
        for joint in self.joint_publishers.keys():
            self.joint_publishers[joint] = self.create_publisher(
                std_msgs.msg.Float64, "/" + joint + "/set_error", 1
            )

        self.get_logger().info("on_configure() successful")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() is called.")

        self.destroy_timer(self.timer)

        for joint in self.joint_publishers.keys():
            self.destroy_publisher(self.joint_publishers[joint])

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")

        self.destroy_timer(self.timer)

        for joint in self.joint_publishers.keys():
            self.destroy_publisher(self.joint_publishers[joint])

        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)

    animation_player_node = AnimationPlayerNode()

    rclpy.spin(animation_player_node)
    animation_player_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
