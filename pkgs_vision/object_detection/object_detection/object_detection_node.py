from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D

from object_detection.src import inference_yolo


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__("object_detection_node")

        # Parameters
        self.declare_parameter("box_size_multiplier", 0.0)
        self.box_size_multiplier = (
            self.get_parameter("box_size_multiplier").get_parameter_value().double_value
        )

        self.declare_parameter("image_w", 1280)
        self.image_w = self.get_parameter("image_w").get_parameter_value().integer_value

        self.declare_parameter("image_h", 720)
        self.image_h = self.get_parameter("image_h").get_parameter_value().integer_value

        self.declare_parameter("publish_annotation", True)
        self.publish_annotation = (
            self.get_parameter("publish_annotation").get_parameter_value().bool_value
        )

        self.declare_parameter("use_compressed", True)
        self.use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
        )

        # Subscriptions
        if self.use_compressed:
            self.subscription = self.create_subscription(
                CompressedImage, "/camera", self.camera_callback, 1
            )
        else:
            self.subscription = self.create_subscription(
                Image, "/camera", self.camera_callback, 1
            )

        # Publishers
        self.publisher_annotation = self.create_publisher(Image, "/camera_annotated", 1)

        self.publisher_bounding_box = self.create_publisher(
            BoundingBox2D, "/target_bounding_box", 1
        )

        # Node variables
        self.class_of_interest = 0  # Class 0 = person in COCO

        self.model = inference_yolo.InferenceYolo(
            "yolov8n.pt", self.image_w, self.image_h, self.box_size_multiplier
        )

        self.cv_bridge = CvBridge()

    def camera_callback(self, msg):

        if self.use_compressed:
            image = self.cv_bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="rgb8"
            )
        else:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Run inference
        results = self.model.run_inference(image)
        box = self.model.compute_target_box(results, self.class_of_interest)

        if not box:
            return

        # Create bounding box message
        box_msg = BoundingBox2D(
            center=Pose2D(
                position=Point2D(x=float(box.x), y=float(box.y)),
                theta=0.0,
            ),
            size_x=float(box.w),
            size_y=float(box.h),
        )

        # Publish bounding box
        self.publisher_bounding_box.publish(box_msg)

        # Publish annotated image
        if self.publish_annotation:

            # Add circle to targeted bounding box on annotated image
            annotated = results[0].plot(show=False)
            cv2.circle(annotated, (int(box.x), int(box.y)), 10, (0, 255, 0), -1)

            image_pub = self.cv_bridge.cv2_to_imgmsg(annotated, encoding="rgb8")

            # Publish annotated image
            self.publisher_annotation.publish(image_pub)


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetectionNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
