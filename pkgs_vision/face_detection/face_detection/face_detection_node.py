from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D

from face_detection.src import inference_yolo


class Box:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def size(self):
        return self.w * self.h


class FaceDetectionNode(Node):

    def __init__(self):
        super().__init__("face_detection_node")

        # Parameters
        self.declare_parameter("inference_device", 1280)
        self.inference_device = (
            self.get_parameter("").get_parameter_value().string_value
        )

        self.declare_parameter("publish_annotation", False)
        self.publish_annotation = (
            self.get_parameter("publish_annotation").get_parameter_value().bool_value
        )

        self.declare_parameter("image_w", 1280)
        self.image_w = self.get_parameter("image_w").get_parameter_value().integer_value

        self.declare_parameter("image_h", 720)
        self.image_h = self.get_parameter("image_h").get_parameter_value().integer_value

        self.declare_parameter("use_compressed", False)
        self.use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
        )

        self.declare_parameter("box_size_multiplier", 0.0)
        self.box_size_multiplier = (
            self.get_parameter("box_size_multiplier").get_parameter_value().double_value
        )

        # Subscriptions
        if self.use_compressed:
            self.subscription = self.create_subscription(
                CompressedImage, "/camera", self.camera_callback, 10
            )
        else:
            self.subscription = self.create_subscription(
                Image, "/camera", self.camera_callback, 10
            )

        # Publishers
        if self.use_compressed:
            self.publisher_annotation = self.create_publisher(
                CompressedImage, "/camera_annotated", 1
            )
        else:
            self.publisher_annotation = self.create_publisher(
                Image, "/camera_annotated", 1
            )

        self.publisher_bounding_box = self.create_publisher(
            BoundingBox2D, "/face_bounding_box", 1
        )

        # Node variables
        model_path = (
            "install/face_detection/share/face_detection/models/yolov8n-face.pt"
        )

        self.model = inference_yolo.InferenceYolo(
            model_path, self.image_w, self.image_h, self.box_size_multiplier
        )

        self.cv_bridge = CvBridge()

    def camera_callback(self, msg):

        if self.use_compressed:
            image = self.cv_bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="rgb8"
            )
        else:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Run face detection model
        results, box = self.model.detect_faces(image)

        # Create bounding box message
        box_msg = BoundingBox2D(
            center=Pose2D(
                position=Point2D(x=box.x, y=box.y),
                theta=0.0,
            ),
            size_x=box.w,
            size_y=box.h,
        )

        # Publish bounding box
        self.publisher_bounding_box.publish(box_msg)

        if self.publish_annotation:

            # Add circle to targeted bounding box on annotated image
            annotated = results[0].plot(show=False)
            cv2.circle(annotated, (int(box.x), int(box.y)), 10, (0, 255, 0), -1)

            if self.use_compressed:
                image_pub = self.cv_bridge.cv2_to_compressed_imgmsg(annotated)
            else:
                image_pub = self.cv_bridge.cv2_to_imgmsg(annotated, encoding="rgb8")

            # Publish annotated image
            self.publisher_annotation.publish(image_pub)


def main(args=None):
    rclpy.init(args=args)

    face_detection_node = FaceDetectionNode()

    rclpy.spin(face_detection_node)
    face_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
