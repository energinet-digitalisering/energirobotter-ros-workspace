from scipy.spatial.transform import Rotation

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from teleoperation.src.vuer_app import VuerApp
from teleoperation.src.vuer_transformer import VuerTransformer
from teleoperation.src.vuer_collision_avoidance import VuerCollisionAvoidance


class TeleoperationNode(Node):

    def __init__(self):
        super().__init__("teleoperation_node")

        # Parameters
        self.declare_parameter("fps", 30)
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value

        self.declare_parameter("camera_enabled", True)
        self.camera_enabled = (
            self.get_parameter("camera_enabled").get_parameter_value().bool_value
        )

        # Subscribers
        self.subscription_image_left = self.create_subscription(
            CompressedImage, "/image_left", self.callback_image_left, 1
        )

        self.subscription_image_right = self.create_subscription(
            CompressedImage, "/image_right", self.callback_image_right, 1
        )

        # Publishers
        self.pose_left_pub = self.create_publisher(
            PoseStamped, "/link_left_hand/target_pose", 1
        )

        self.pose_right_pub = self.create_publisher(
            PoseStamped, "/link_right_hand/target_pose", 1
        )

        # Timers
        self.timer = self.create_timer(1.0 / self.fps, self.callback_timer)

        # Variables
        self.image_left = None
        self.image_right = None

        self.cv_bridge = CvBridge()

        self.vuer_app = VuerApp(self.camera_enabled)
        self.vuer_transformer = VuerTransformer()
        self.vuer_collision_avoidance = VuerCollisionAvoidance()

    def callback_image_left(self, msg):
        self.image_left = self.cv_bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding="rgb8"
        )

    def callback_image_right(self, msg):
        self.image_right = self.cv_bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding="rgb8"
        )

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

    def callback_timer(self):
        self.vuer_app.update_frames(self.image_left, self.image_right)

        _, left_wrist_mat, right_wrist_mat = self.vuer_transformer.process(
            self.vuer_app
        )
        left_wrist_mat, right_wrist_mat = self.vuer_collision_avoidance.process(
            left_wrist_mat, right_wrist_mat
        )

        msg_pose_left = self.tf_matrix_to_msg(left_wrist_mat)
        self.pose_left_pub.publish(msg_pose_left)

        msg_pose_right = self.tf_matrix_to_msg(right_wrist_mat)
        self.pose_right_pub.publish(msg_pose_right)


def main(args=None):
    rclpy.init(args=args)

    teleoperation_node = TeleoperationNode()

    rclpy.spin(teleoperation_node)
    teleoperation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
