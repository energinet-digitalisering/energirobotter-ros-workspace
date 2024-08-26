from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D


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
        self.publisher_annotation = self.create_publisher(Image, "/camera_annotated", 1)

        self.publisher_bounding_box = self.create_publisher(
            BoundingBox2D, "/face_bounding_box", 1
        )

        # Node variables
        model_path = "models/yolov8n-face.pt"
        self.model = YOLO(model_path)
        self.box_prev = Box(self.image_w / 2, self.image_h / 2, 0, 0)

        self.cv_bridge = CvBridge()

    def box_closest(self, target, boxes):

        distance_min = 2 * self.image_w
        box = None
        for x, y, w, h in boxes:
            distance = np.linalg.norm([target[0] - x, target[1] - y])

            if distance < distance_min:
                distance_min = distance
                box = Box(x, y, w, h)

        return box

    def box_largest(self, boxes):

        largest_size = 0
        box = None
        for x, y, w, h in boxes:
            size = w * h

            if size > (largest_size):
                largest_size = size
                box = Box(x, y, w, h)

        return box

    def camera_callback(self, msg):

        if self.use_compressed:
            image = self.cv_bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="rgb8"
            )
        else:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Run face detection model
        results = self.model.predict(source=image, show=False, verbose=False)
        annotated = results[0].plot(show=False)

        # Convert results to list of boxes
        boxes = []
        for result in results:
            for box in result.boxes.xywh.tolist():
                boxes.append(box)

        if len(boxes) != 0:  # Ensure detection

            box_closest = self.box_closest((self.box_prev.x, self.box_prev.y), boxes)
            box_largest = self.box_largest(boxes)

            box_size_buffer = box_closest.size() * self.box_size_multiplier

            box_target = (
                box_largest
                if (box_largest.size() > (box_closest.size() + box_size_buffer))
                else box_closest
            )

            self.box_prev = box_target

            # Create bounding box message
            box_msg = BoundingBox2D(
                center=Pose2D(
                    position=Point2D(x=box_target.x, y=box_target.y),
                    theta=0.0,
                ),
                size_x=box_target.w,
                size_y=box_target.h,
            )

            # Publish bounding box
            self.publisher_bounding_box.publish(box_msg)

            # Add circle to targeted bounding box on annotated image
            cv2.circle(
                annotated, (int(box_target.x), int(box_target.y)), 10, (0, 255, 0), -1
            )

        # Publish annotated image
        self.publisher_annotation.publish(
            self.cv_bridge.cv2_to_imgmsg(annotated, encoding="rgb8")
        )


def main(args=None):
    rclpy.init(args=args)

    face_detection_node = FaceDetectionNode()

    rclpy.spin(face_detection_node)
    face_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
