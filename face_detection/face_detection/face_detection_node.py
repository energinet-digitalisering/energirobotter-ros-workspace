from cv_bridge import CvBridge
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class FaceDetectionNode(Node):

    def __init__(self):
        super().__init__("face_detection_node")

        # Publishers
        self.publisher = self.create_publisher(Image, "/camera_annotated", 1)

        # Subscriptions
        self.subscription = self.create_subscription(
            Image, "/camera", self.camera_callback, 1
        )

        # Node variables
        model_path = (
            "install/face_recognition/share/face_recognition/models/yolov8n-face.pt"
        )
        self.model = YOLO(model_path)

        self.cv_bridge = CvBridge()

    def camera_callback(self, msg):
        self.get_logger().info("Image recieved")

        image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        results = self.model.predict(source=image, show=False)
        annotated = results[0].plot(show=False)

        self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)

    face_detection_node = FaceDetectionNode()

    rclpy.spin(face_detection_node)
    face_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
