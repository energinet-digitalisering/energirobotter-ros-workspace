import json
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState

from teleoperation.src.zeromq_app import ZeroMQApp
from teleoperation.src.tracking_transformer import TrackingTransformer
from teleoperation.src.tracking_filter import TrackingFilter
from teleoperation.src.tracking_collision_avoidance import TrackingCollisionAvoidance


class TeleoperationNode(Node):

    def __init__(self):
        super().__init__("teleoperation_node")

        # Parameters
        self.declare_parameter("fps", 30)
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value

        # Subscribers
        self.sub_tracking = self.create_subscription(
            String, "/tracking_data", self.callback_tracking, 1
        )

        # Publishers
        self.pub_pose_left = self.create_publisher(
            PoseStamped, "/link_left_hand/target_pose", 1
        )

        self.pub_pose_right = self.create_publisher(
            PoseStamped, "/link_right_hand/target_pose", 1
        )

        self.pub_joint_state_hands = self.create_publisher(
            JointState, "/joint_states_hands", 1
        )

        # Timers
        self.timer = self.create_timer(1.0 / self.fps, self.callback_timer)

        # Variables
        self.zeromq_app = ZeroMQApp()
        self.tracking_transformer = TrackingTransformer()
        self.tracking_filter_left = TrackingFilter()
        self.tracking_filter_right = TrackingFilter()
        self.tracking_collision_avoidance = TrackingCollisionAvoidance()

    def callback_tracking(self, msg):
        tracking_dict = json.loads(msg.data)
        self.zeromq_app.update_tracking(tracking_dict)

    def tf_matrix_to_msg(self, tf_matrix):
        position = tf_matrix[0:3, 3]

        rotation_matrix = tf_matrix[0:3, 0:3]
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()

        msg = PoseStamped(
            header=Header(frame_id="link_torso", stamp=self.get_clock().now().to_msg()),
            pose=Pose(
                position=Point(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(
                    x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
                ),
            ),
        )

        return msg

    def dict_to_joint_state_msg(self, dict):
        names = list(dict.keys())
        positions = list(dict.values())

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = names
        joint_state_msg.position = positions

        return joint_state_msg

    def callback_timer(self):
        # Transform tracking to robot frame
        _, left_wrist_mat, right_wrist_mat, hand_angles = (
            self.tracking_transformer.process(
                self.zeromq_app.head_matrix,
                self.zeromq_app.hand_left,
                self.zeromq_app.hand_right,
            )
        )

        # Filter raw tracking
        left_wrist_mat = self.tracking_filter_left.low_pass(left_wrist_mat)
        right_wrist_mat = self.tracking_filter_right.low_pass(right_wrist_mat)

        # Avoid hand collision
        left_wrist_mat, right_wrist_mat = self.tracking_collision_avoidance.process(
            left_wrist_mat, right_wrist_mat
        )

        msg_pose_left = self.tf_matrix_to_msg(left_wrist_mat)
        self.pub_pose_left.publish(msg_pose_left)

        msg_pose_right = self.tf_matrix_to_msg(right_wrist_mat)
        self.pub_pose_right.publish(msg_pose_right)

        msg_angles_hands = self.dict_to_joint_state_msg(hand_angles)
        self.pub_joint_state_hands.publish(msg_angles_hands)


def main(args=None):
    rclpy.init(args=args)

    teleoperation_node = TeleoperationNode()

    rclpy.spin(teleoperation_node)
    teleoperation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
