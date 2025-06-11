import matplotlib.pyplot as plt
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import deque


class JointStatePlotter(Node):
    def __init__(self):
        super().__init__("joint_state_plotter")
        self.subscription = self.create_subscription(
            JointState, "/log_speeds", self.joint_state_callback, 1
        )

        self.joint_names = []
        self.joint_data = {}
        self.history_length = 100  # Number of data points to keep

        self.fig, self.ax = plt.subplots()
        self.lines = {}
        self.ax.set_title("Joint States Over Time")
        self.ax.set_xlabel("Time Step")
        self.ax.set_ylabel("Position")
        self.ax.set_xlim(0, self.history_length)

        self.timer = self.create_timer(
            0.1, self.update_plot
        )  # ROS timer to update plot

    def joint_state_callback(self, msg):
        if not self.joint_names:
            self.joint_names = msg.name
            for name in self.joint_names:
                self.joint_data[name] = deque(maxlen=self.history_length)
                (self.lines[name],) = self.ax.plot(
                    [], [], label=name
                )  # Initialize lines
            self.ax.legend()
            self.fig.canvas.draw_idle()

        for i, name in enumerate(msg.name):
            if name in self.joint_data:
                self.joint_data[name].append(msg.position[i])

    def update_plot(self):
        for name, line in self.lines.items():
            if name in self.joint_data and len(self.joint_data[name]) > 0:
                line.set_data(
                    range(len(self.joint_data[name])), list(self.joint_data[name])
                )

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.legend()  # Ensure legend updates dynamically
        self.fig.canvas.draw_idle()
        plt.pause(0.01)  # Allow ROS to process callbacks


def main(args=None):
    rclpy.init(args=args)
    plotter = JointStatePlotter()
    try:
        rclpy.spin(plotter)  # Keep ROS node running
    except KeyboardInterrupt:
        pass
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
