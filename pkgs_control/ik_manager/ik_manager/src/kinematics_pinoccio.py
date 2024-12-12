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

        target_pose = pinocchio.SE3(target_rotation, target_position)

        if self.model.existFrame(end_effector_frame):
            frame_id = self.model.getFrameId(end_effector_frame)

            # Perform inverse kinematics
            success, q_solution = self.compute_ik(frame_id, target_pose)

            if q_solution is not None:
                # self.logger.info(f"IK solution found: {q_solution}")
                return success, q_solution
            else:
                self.logger.error("Failed to compute IK solution.")
        else:
            self.logger.error(f"Frame '{end_effector_frame}' not found in model!")

    def compute_ik(self, frame_id, target_pose, tol=1e-4, max_iter=1000):
        """
        Compute the inverse kinematics for a given end-effector pose.

        Args:
            frame_id: Frame ID of the end-effector.
            target_pose: Desired SE3 pose of the end-effector.
            tol: Tolerance for position/orientation error.
            max_iter: Maximum number of iterations.

        Returns:
            Solution joint configuration or None if no solution is found.
        """

        joint_id = self.model.frames[frame_id].parent
        q = self.q_init.copy()

        # world_to_target = target_pose
        # world_to_end_effector = self.data.oMi[joint_id]
        # # Transform target pose to match the reference frame of the end-effector
        # local_target_pose = world_to_end_effector.actInv(world_to_target)

        DT = 1e-1
        damp = 1e-12

        i = 0
        while True:
            pinocchio.forwardKinematics(self.model, self.data, q)

            # DEBUG
            if i == 0:
                self.logger.info(f"Init Pose = {self.data.oMi[joint_id].translation.T}")
            # DEBUG END

            dMi = target_pose.actInv(self.data.oMi[joint_id])

            err = pinocchio.log(dMi).vector
            position_error = err[:3]
            orientation_error = err[3:]
            err_weighted = np.hstack((position_error, 0.1 * orientation_error))

            if np.linalg.norm(err_weighted) < tol:
                success = True
                break
            if i >= max_iter:
                success = False
                break

            J = pinocchio.computeJointJacobian(self.model, self.data, q, joint_id)
            v = -J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err_weighted))
            q = pinocchio.integrate(self.model, q, v * DT)

            # if not i % 10:
            #     self.logger.info("%d: error = %s" % (i, err.T))

            i += 1

        # self.logger.info("\nresult: %s" % q.flatten().tolist())
        # self.logger.info("\nfinal error: %s" % err.T)

        # self.logger.info(
        #     f"Error = {err_weighted.T}, Pose = {self.data.oMi[joint_id].translation.T}"
        # )

        if success:
            self.logger.info("Convergence achieved!")
        else:
            self.logger.warning(
                "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
            )

        return success, q

        # for i in range(max_iter):
        #     # Compute current pose of the end-effector
        #     pinocchio.forwardKinematics(self.model, self.data, q)
        #     pinocchio.updateFramePlacements(self.model, self.data)
        #     current_pose = self.data.oMf[frame_id]

        #     # Compute pose error
        #     position_error = desired_pose.translation - current_pose.translation
        #     rotation_error = 0.5 * (
        #         np.cross(current_pose.rotation[:, 0], desired_pose.rotation[:, 0])
        #         + np.cross(current_pose.rotation[:, 1], desired_pose.rotation[:, 1])
        #         + np.cross(current_pose.rotation[:, 2], desired_pose.rotation[:, 2])
        #     )
        #     error = np.concatenate([position_error, rotation_error])

        #     # Check for convergence
        #     if np.linalg.norm(error) < tol:
        #         return q

        #     # Compute Jacobian
        #     J = pinocchio.computeFrameJacobian(self.model, self.data, q, frame_id)
        #     J_pos_orient = J[:6]  # Use both position and orientation components

        #     # Compute joint velocity update
        #     dq = np.linalg.pinv(J_pos_orient).dot(error)

        #     # Update joint configuration
        #     q = pinocchio.integrate(self.model, q, dq)

        # # Return None if no solution is found within max_iter
        # return None


if __name__ == "__main__":

    ik_solver = KinematicsPinoccio(
        urdf_path="install/elrik_description/share/elrik_description/urdf/phobos_generated.urdf"
    )

    ik_solver.perform_ik("link_left_hand", np.array([0.0, 0.0, 0.0]), np.eye(3))
