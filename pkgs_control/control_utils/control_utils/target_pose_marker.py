#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class TargetPoseMarker(Node):
    def __init__(self):
        super().__init__("target_pose_marker")

        # Publisher for target pose
        self.pub = self.create_publisher(PoseStamped, "/left/target_pose", 10)

        # Interactive Marker Server
        self.server = InteractiveMarkerServer(self, "target_pose_marker")

        # Create interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "link_torso"  # base frame of your robot
        int_marker.name = "left_target"
        int_marker.description = "6-DoF Target Pose"
        int_marker.pose.position.x = -0.2
        int_marker.pose.position.y = 0.5
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.w = 1.0

        # --- Add small visual sphere ---
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.05  # 5 cm
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 0.5
        sphere_marker.color.b = 1.0
        sphere_marker.color.a = 0.8

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(sphere_marker)
        int_marker.controls.append(visual_control)

        # --- 6-DoF move controls ---
        for axis in ["x", "y", "z"]:
            control = InteractiveMarkerControl()
            control.name = f"move_{axis}"
            control.orientation.w = 1.0
            if axis == "x":
                control.orientation.x = 1.0
            if axis == "y":
                control.orientation.y = 1.0
            if axis == "z":
                control.orientation.z = 1.0
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)

        # --- 6-DoF rotate controls ---
        for axis in ["x", "y", "z"]:
            control = InteractiveMarkerControl()
            control.name = f"rotate_{axis}"
            control.orientation.w = 1.0
            if axis == "x":
                control.orientation.x = 1.0
            if axis == "y":
                control.orientation.y = 1.0
            if axis == "z":
                control.orientation.z = 1.0
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control)

        # Insert into server
        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = feedback.header.frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose = feedback.pose
        self.pub.publish(pose_msg)

        quat = feedback.pose.orientation
        self.get_logger().info(f"Rotation: {quat.x}, {quat.y}, {quat.z}, {quat.w}")


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseMarker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
