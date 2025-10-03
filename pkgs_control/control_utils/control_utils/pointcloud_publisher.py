import open3d as o3d
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class PointCloudPublisher(Node):
    def __init__(self, ply_file: str):
        super().__init__("pointcloud_publisher")
        self.publisher_ = self.create_publisher(PointCloud2, "reachability_cloud", 10)

        # Load PLY
        pcd = o3d.io.read_point_cloud(ply_file)
        self.points = np.asarray(pcd.points, dtype=np.float32)

        # Timer
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(
            f"Publishing point cloud with {len(self.points)} points..."
        )

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "link_torso"

        msg = pc2.create_cloud_xyz32(header, self.points.tolist())
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher("pointcloud.ply")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
