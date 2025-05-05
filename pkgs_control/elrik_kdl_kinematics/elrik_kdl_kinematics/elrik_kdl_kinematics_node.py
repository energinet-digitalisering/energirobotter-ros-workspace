from typing import List

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .kdl_kinematics import (
    generate_solver,
    inverse_kinematics,
    ros_pose_to_matrix,
)


class ElrikKdlKinematics(Node):
    def __init__(self):
        super().__init__("elrik_kdl_kinematics_node")

        self.urdf = self.retrieve_urdf()

        self.end_effectors = ["link_left_hand", "link_right_hand", "link_head_roll"]

        self.chain_names = {
            self.end_effectors[0]: "left",
            self.end_effectors[1]: "right",
            self.end_effectors[2]: "head",
        }


        self.end_effector_callback_subs = {
            self.end_effectors[0]: self.callback_target_pos_left,
            self.end_effectors[1]: self.callback_target_pos_right,
            self.end_effectors[2]: self.callback_target_pos_head,
        }

        self.target_pose = {
            self.end_effectors[0]: np.array(
                [[1, 0, 0, 0.5], [0, 1, 0, 0.5], [0, 0, 1, 0.5], [0, 0, 0, 1]]
            ),
            self.end_effectors[1]: np.array(
                [[1, 0, 0, 0.5], [0, 1, 0, 0.5], [0, 0, 1, 0.5], [0, 0, 0, 1]]
            ),
            self.end_effectors[2]: np.array(
                [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            ),
        }

        self.q_init = {}
        self.chain, self.fk_solver, self.ik_solver = {}, {}, {}
        self.target_sub = {}

        self.timer = self.create_timer(0.1, self.callback_timer_publish_joint_states)

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        for end_effector in self.end_effectors:

            chain, fk_solver, ik_solver = generate_solver(
                self.urdf, "link_torso", end_effector
            )

            # We automatically loads the kinematics corresponding to the config
            if chain.getNrOfJoints():
                self.get_logger().info(f'Found kinematics chain for "{end_effector}"!')

                self.target_sub[end_effector] = self.create_subscription(
                    msg_type=PoseStamped,
                    topic=f"/{self.chain_names[end_effector]}/target_pose",
                    qos_profile=5,
                    callback=self.end_effector_callback_subs[end_effector],
                )
                self.get_logger().info(
                    f'Added subscription on "{self.target_sub[end_effector].topic}"'
                )

                self.q_init[end_effector] = [0] * chain.getNrOfJoints()
                self.chain[end_effector] = chain
                self.fk_solver[end_effector] = fk_solver
                self.ik_solver[end_effector] = ik_solver

        self.get_logger().info(f"Kinematics node ready!")

    def callback_target_pos_left(self, msg: PoseStamped):
        self.target_pose[self.end_effectors[0]] = ros_pose_to_matrix(msg.pose)

    def callback_target_pos_right(self, msg: PoseStamped):
        self.target_pose[self.end_effectors[1]] = ros_pose_to_matrix(msg.pose)

    def callback_target_pos_head(self, msg: PoseStamped):
        self.target_pose[self.end_effectors[2]] = ros_pose_to_matrix(msg.pose)

    def callback_timer_publish_joint_states(self):
        """
        Publish the joint states based on the IK solution.
        """
        names = []
        positions = []

        for end_effector in self.end_effectors:

            if end_effector != self.end_effectors[2]:  # Skip head
                error, q_solution = inverse_kinematics(
                    self.ik_solver[end_effector],
                    q0=self.q_init[end_effector],
                    target_pose=self.target_pose[end_effector],
                    nb_joints=self.chain[end_effector].getNrOfJoints(),
                    locked_joints={5: 0.0},
                )
            else:
                q_solution = self.q_init[end_effector]

            names.extend(self.get_chain_joints_name(self.chain[end_effector]))
            positions.extend([float(pos) for pos in q_solution])

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = names
        joint_state_msg.position = positions

        # Publish the joint state message
        self.joint_state_pub.publish(joint_state_msg)

    def get_current_position(self, chain) -> List[float]:
        joints = self.get_chain_joints_name(chain)
        return [self._current_pos[j] for j in joints]

    def wait_for_joint_state(self):
        while not self.joint_state_ready.is_set():
            self.get_logger().info("Waiting for /joint_states...")
            rclpy.spin_once(self)

    def retrieve_urdf(self, timeout_sec: float = 15):
        self.get_logger().info('Retrieving URDF from "/robot_description"...')

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.urdf = None

        def urdf_received(msg: String):
            self.urdf = msg.data

        self.create_subscription(
            msg_type=String,
            topic="/robot_description",
            qos_profile=qos_profile,
            callback=urdf_received,
        )
        rclpy.spin_once(self, timeout_sec=timeout_sec)

        if self.urdf is None:
            self.get_logger().error("Could not retrieve the URDF!")
            raise EnvironmentError("Could not retrieve the URDF!")

        self.get_logger().info("Done!")

        return self.urdf

    def check_position(self, js: JointState, chain) -> List[float]:
        pos = dict(zip(js.name, js.position))
        try:
            joints = [pos[j] for j in self.get_chain_joints_name(chain)]
            return joints
        except KeyError:
            self.get_logger().warning(
                f"Incorrect joints found ({js.name} vs {self.get_chain_joints_name(chain)})"
            )
            raise

    def get_chain_joints_name(self, chain):
        return [
            chain.getSegment(i).getJoint().getName()
            for i in range(chain.getNrOfJoints())
        ]


def main():
    rclpy.init()
    node = ElrikKdlKinematics()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
