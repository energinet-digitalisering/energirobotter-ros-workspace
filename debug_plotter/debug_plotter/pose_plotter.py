import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from collections import deque


class PoseStampedPlotter(Node):
    def __init__(self):
        super().__init__("pose_stamped_plotter")
        self.subscription = self.create_subscription(
            PoseStamped, "/link_left_hand/target_pose", self.callback_pose, 1
        )

        self.position_labels = ["x", "y", "z"]
        self.position_data = {
            label: deque(maxlen=100) for label in self.position_labels
        }
        self.history_length = 100

        self.fig, self.ax = plt.subplots()
        self.lines = {
            label: self.ax.plot([], [], label=label)[0]
            for label in self.position_labels
        }
        self.ax.set_title("PoseStamped Position Over Time")
        self.ax.set_xlabel("Time Step")
        self.ax.set_ylabel("Position")
        self.ax.set_xlim(0, self.history_length)
        self.ax.legend()

        self.timer = self.create_timer(0.1, self.update_plot)

    def callback_pose(self, msg):
        self.position_data["x"].append(msg.pose.position.x)
        self.position_data["y"].append(msg.pose.position.y)
        self.position_data["z"].append(msg.pose.position.z)

    def update_plot(self):
        for label, line in self.lines.items():
            if len(self.position_data[label]) > 0:
                line.set_data(
                    range(len(self.position_data[label])),
                    list(self.position_data[label]),
                )

        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw_idle()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    plotter = PoseStampedPlotter()
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
