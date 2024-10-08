from typing import Optional

import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
import std_msgs


class AnimationPlayerNode(Node):

    def __init__(self):
        super().__init__("animation_player_node")

        # Parameters

        self.declare_parameter("fps", 24)
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value

        self.declare_parameter("csv_file_path", 24)
        self.csv_file_path = (
            self.get_parameter("csv_file_path").get_parameter_value().string_value
        )

        # Lifecycle Node timers and publishers
        self.timer: Optional[Timer] = None
        self.publisher_joint: Optional[Publisher] = None

        # Node variables

    ##################### Callbacks #####################

    def callback_timer(self): ...

    ##################### Lifecyle Node Functions #####################

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        # Publishers
        self.publisher_joint = self.create_publisher(
            std_msgs.msg.Float64, "/servo_0/set_angle", 1
        )

        # Timers
        self.timer = self.create_timer(1 / self.fps, self.callback_timer)

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
        self.destroy_publisher(self.publisher_joint)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")

        self.destroy_timer(self.timer)
        self.destroy_publisher(self.publisher_joint)

        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)

    animation_player_node = AnimationPlayerNode()

    rclpy.spin(animation_player_node)
    animation_player_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
