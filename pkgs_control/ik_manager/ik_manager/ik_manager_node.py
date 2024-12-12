import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

from ik_manager.src import kinematics_pinoccio


class IKManagerNode(Node):
    def __init__(self):
        super().__init__("ik_manager_node")

        # Subscriptions
        self.target_pos_sub = self.create_subscription(
            Vector3,
            "/target_pos",
            self.callback_target_pos,
            1,
        )

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Timers
        self.timer = self.create_timer(0.5, self.callback_publish_joint_states)

        # Node variables
        self.target_pos = [-0.25601042, -0.01441534, 0.58050495]
        # self.target_pos = [-1.5, -1.5, -1.5]
        valid_pos_1 = [-0.3, -0.2, 1.2]
        self.init_pose = [-0.25601042, -0.01441534, 0.58050495]

        self.ik_solver = kinematics_pinoccio.KinematicsPinoccio(
            urdf_path="install/elrik_description/share/elrik_description/urdf/phobos_generated.urdf"
        )

        self.generator = self.combination_generator()

    ##################### Callbacks #####################

    def callback_target_pos(self, msg):
        self.get_logger().info(f"Updating target position: [{msg.x}, {msg.y}, {msg.z}]")
        self.target_pos = [msg.x, msg.y, msg.z]

    def callback_publish_joint_states(self):
        """
        Publish the joint states based on the IK solution.
        """

        # self.get_logger().info(
        #     f"x: {self.target_pos[0]}, y: {self.target_pos[1]}, z: {self.target_pos[2]}"
        # )

        joint_names = self.ik_solver.get_joint_names()

        # Perform IK
        success, q_solution = self.ik_solver.perform_ik(
            "link_right_hand", np.array(self.target_pos), np.eye(3)
        )

        if q_solution is None:
            q_solution = self.ik_solver.q_init

        expected_joint_count = len(joint_names)
        q_positions = q_solution[:expected_joint_count]

        if len(q_positions) != expected_joint_count:
            self.get_logger().error(
                f"Mismatch between joint names ({expected_joint_count}) and joint positions ({len(q_positions)})."
            )
            return

        # Create message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = joint_names
        joint_state_msg.position = [float(pos) for pos in q_positions]

        # Publish the joint state message
        self.joint_state_pub.publish(joint_state_msg)

        # if not success:
        #     self.target_pos = next(self.generator)

    def combination_generator(self, start=-1.5, end=1.5, step=0.1):
        """
        Generator that yields combinations of three numbers in the interval [start, end]
        with the specified step size.

        Args:
            start (float): Start of the interval (inclusive).
            end (float): End of the interval (inclusive).
            step (float): Step size for the interval.

        Yields:
            tuple: The next combination of three numbers.
        """
        values = np.arange(start, end + step, step)
        for x in values:
            for y in values:
                for z in values:
                    yield (x, y, z)


def main(args=None):
    rclpy.init(args=args)
    node = IKManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
