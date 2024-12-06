import logging
import numpy as np
import pinocchio


class KinematicsPinoccio:
    def __init__(self, urdf_path):

        self.logger = logging.getLogger("KinematicsPinoccio")
        logging.basicConfig(level=logging.INFO)

        # Load the robot model

        try:
            self.model = pinocchio.buildModelFromUrdf(urdf_path)
        except Exception as e:
            self.logger.error(f"Failed to load URDF: {e}")
            raise

        self.data = self.model.createData()

        # Set a neutral configuration (initial joint angles)
        self.q_init = pinocchio.neutral(self.model)

    def get_joint_names(self):
        # Filter out non-joint entries from model.names (e.g., the root link)
        joint_names = [name for name in self.model.names[1:] if isinstance(name, str)]
        return joint_names

    def perform_ik(self, end_effector_frame, target_position, target_rotation):

        desired_pose = pinocchio.SE3(target_rotation, target_position)

        if self.model.existFrame(end_effector_frame):
            frame_index = self.model.getFrameId(end_effector_frame)

            # Perform inverse kinematics
            q_solution = self.compute_ik(
                self.model, self.data, self.q_init, frame_index, desired_pose
            )

            if q_solution is not None:
                self.logger.info(f"IK solution found: {q_solution}")
                return q_solution
            else:
                self.logger.error("Failed to compute IK solution.")
        else:
            self.logger.error(f"Frame '{end_effector_frame}' not found in model!")

    def compute_ik(
        self, model, data, q_init, frame_index, desired_pose, tol=0.5, max_iter=1000
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


if __name__ == "__main__":

    ik_solver = KinematicsPinoccio(
        urdf_path="install/elrik_description/share/elrik_description/urdf/phobos_generated.urdf"
    )

    ik_solver.perform_ik("link_left_hand", np.array([0.0, 0.0, 0.0]), np.eye(3))
