from dataclasses import dataclass, field
import numpy as np
from typing import Callable, Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R

from .kdl_kinematics import (
    generate_solver,
    inverse_kinematics,
    ros_pose_to_matrix,
)


@dataclass
class EndEffector:
    name: str
    callback: Callable
    locked_joints: Dict[int, float] = field(
        default_factory=dict
    )  # {link_id(int), angle(float)}
    target_pose: np.ndarray = field(
        default_factory=lambda: np.array(
            [[1, 0, 0, 0.5], [0, 1, 0, 0.5], [0, 0, 1, 0.5], [0, 0, 0, 1]]
        )
    )
    q_init = {}
    chain = {}
    fk_solver = {}
    ik_solver = {}


class ElrikKdlKinematics(Node):
    def __init__(self):
        super().__init__("elrik_kdl_kinematics_node")

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Timers
        self.timer = self.create_timer(0.1, self.callback_timer_publish_joint_states)

        # Node variables
        self.urdf = self.retrieve_urdf()

        self.end_effectors = {
            "left": EndEffector(
                name="link_left_hand",
                callback=self.callback_target_pos_left,
            ),
            "right": EndEffector(
                name="link_right_hand",
                callback=self.callback_target_pos_right,
            ),
        }

        self.target_subs = []

        # Subscribe to head pose
        self.head_pose = None
        self.create_subscription(
            PoseStamped, "/head/target_pose", self.callback_head_pose, 5
        )

        # Init IK and subscriptions for end effectors
        for key, end_effector in self.end_effectors.items():

            chain, fk_solver, ik_solver = generate_solver(
                self.urdf, "link_torso", end_effector.name
            )

            # We automatically loads the kinematics corresponding to the config
            if chain.getNrOfJoints():
                self.get_logger().info(
                    f'Found kinematics chain for "{end_effector.name}"! Chain length: {chain.getNrOfJoints()}'
                )

                target_sub = self.create_subscription(
                    msg_type=PoseStamped,
                    topic=f"/{key}/target_pose",
                    qos_profile=5,
                    callback=end_effector.callback,
                )
                self.target_subs.append(target_sub)

                self.get_logger().info(f'Added subscription on "{target_sub.topic}"')

                end_effector.q_init = [0] * chain.getNrOfJoints()
                end_effector.chain = chain
                end_effector.fk_solver = fk_solver
                end_effector.ik_solver = ik_solver

        self.get_logger().info(f"Kinematics node ready!")

    ############## Functions ##############

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

    def get_chain_joints_name(self, chain):
        return [
            chain.getSegment(i).getJoint().getName()
            for i in range(chain.getNrOfJoints())
        ]

    ############## Callbacks ##############

    def callback_target_pos_left(self, msg: PoseStamped):
        self.end_effectors["left"].target_pose = ros_pose_to_matrix(msg.pose)

    def callback_target_pos_right(self, msg: PoseStamped):
        self.end_effectors["right"].target_pose = ros_pose_to_matrix(msg.pose)

    def callback_head_pose(self, msg: PoseStamped):
        """
        Extract yaw and pitch from the head pose transformation matrix (in radians).
        """
        matrix = ros_pose_to_matrix(msg.pose)
        # Extract rotation part (3x3)
        rot_matrix = matrix[:3, :3]
        # Convert to Euler angles (roll, pitch, yaw)
        euler = R.from_matrix(rot_matrix).as_euler("zyx", degrees=False)
        yaw, roll, pitch = euler
        self.head_pose = {"yaw": yaw, "pitch": pitch}

    def callback_timer_publish_joint_states(self):
        """
        Publish the joint states based on the IK solution, including head joints.
        """
        names = []
        positions = []

        for end_effector in self.end_effectors.values():

            error, q_solution = inverse_kinematics(
                end_effector.ik_solver,
                q0=end_effector.q_init,
                target_pose=end_effector.target_pose,
                nb_joints=end_effector.chain.getNrOfJoints(),
                locked_joints=end_effector.locked_joints,
            )

            names.extend(self.get_chain_joints_name(end_effector.chain))
            positions.extend([float(pos) for pos in q_solution])

        # Add head joints if head pose is available
        if self.head_pose is not None:
            names.extend(["joint_head_yaw", "joint_head_pitch"])
            positions.extend(
                [float(self.head_pose["yaw"]), float(self.head_pose["pitch"])]
            )

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = names
        joint_state_msg.position = positions

        # Publish the joint state message
        self.joint_state_pub.publish(joint_state_msg)


def main():
    rclpy.init()
    node = ElrikKdlKinematics()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
