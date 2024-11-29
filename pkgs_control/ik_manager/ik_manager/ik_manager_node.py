import rclpy
from rclpy.node import Node
import pinocchio
import numpy as np


class IKManagerNode(Node):
    def __init__(self):
        super().__init__("ik_manager_node")

        # Load the robot model
        model = pinocchio.buildModelFromUrdf(
            "install/elrik_description/share/elrik_description/urdf/elrik.urdf"
        )
        data = model.createData()

        # Set a neutral configuration (initial joint angles)
        q = pinocchio.neutral(model)

        # Desired end-effector pose (example)
        desired_position = np.array([0.1, 0.2, 0.3])  # XYZ target
        desired_rotation = np.eye(3)  # Identity rotation matrix
        desired_pose = pinocchio.SE3(desired_rotation, desired_position)

        # Perform inverse kinematics
        frame_name = "left_hand_link"  # End-effector frame
        if model.existFrame(frame_name):
            frame_index = model.getFrameId(frame_name)
            q_solution = self.compute_ik(model, data, q, frame_index, desired_pose)

            if q_solution is not None:
                self.get_logger().info(f"IK solution found: {q_solution}")
                # Update forward kinematics with new solution
                pinocchio.forwardKinematics(model, data, q_solution)
                pinocchio.updateFramePlacements(model, data)
                updated_pose = data.oMf[frame_index]
                self.get_logger().info(
                    f"Updated End-effector position: {updated_pose.translation}"
                )
                for i, angle in enumerate(q_solution):
                    self.get_logger().info(
                        f"Joint {model.names[i]}: {np.rad2deg(angle)}"
                    )

                # for i, name in enumerate(model.names):
                # self.get_logger().info(f"Index {i}: {name}")

            else:
                self.get_logger().error("Failed to compute IK solution.")
        else:
            self.get_logger().error(f"Frame '{frame_name}' not found in model!")

    def compute_ik(
        self, model, data, q_init, frame_index, desired_pose, tol=1e-4, max_iter=100
    ):
        """
        Compute the inverse kinematics for a given end-effector pose.

        Args:
            model: Pinocchio model of the robot.
            data: Pinocchio data object.
            q_init: Initial joint configuration.
            frame_index: Frame index of the end-effector.
            desired_pose: Desired SE3 pose of the end-effector.
            tol: Tolerance for position/orientation error.
            max_iter: Maximum number of iterations.

        Returns:
            Solution joint configuration or None if no solution is found.
        """
        q = q_init.copy()
        for i in range(max_iter):
            # Compute current pose of the end-effector
            pinocchio.forwardKinematics(model, data, q)
            pinocchio.updateFramePlacements(model, data)
            current_pose = data.oMf[frame_index]

            # Compute pose error
            position_error = desired_pose.translation - current_pose.translation
            rotation_error = 0.5 * (
                np.cross(current_pose.rotation[:, 0], desired_pose.rotation[:, 0])
                + np.cross(current_pose.rotation[:, 1], desired_pose.rotation[:, 1])
                + np.cross(current_pose.rotation[:, 2], desired_pose.rotation[:, 2])
            )
            error = np.concatenate([position_error, rotation_error])

            # Check for convergence
            if np.linalg.norm(error) < tol:
                return q

            # Compute Jacobian
            J = pinocchio.computeFrameJacobian(model, data, q, frame_index)
            J_pos_orient = J[:6]  # Use both position and orientation components

            # Compute joint velocity update
            dq = np.linalg.pinv(J_pos_orient).dot(error)

            # Update joint configuration
            q = pinocchio.integrate(model, q, dq)

        # Return None if no solution is found within max_iter
        return None


def main(args=None):
    rclpy.init(args=args)
    node = IKManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
