from cv_bridge import CvBridge
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage


class FaceDetectionNode(Node):

    def __init__(self):
        super().__init__("face_detection_node")

        # Parameters
        self.declare_parameter("use_compressed", False)
        self.use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
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
        self.publisher = self.create_publisher(Image, "/camera_annotated", 1)

        # Node variables
        model_path = "models/yolov8n-face.pt"
        self.model = YOLO(model_path)

        self.cv_bridge = CvBridge()

    def camera_callback(self, msg):

        if self.use_compressed:
            image = self.cv_bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="rgb8"
            )
        else:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        results = self.model.predict(source=image, show=False)
        annotated = results[0].plot(show=False)

        self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(annotated, encoding="rgb8"))


def main(args=None):
    rclpy.init(args=args)

    face_detection_node = FaceDetectionNode()

    rclpy.spin(face_detection_node)
    face_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
