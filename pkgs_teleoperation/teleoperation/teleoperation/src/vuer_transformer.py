import numpy as np
import logging


JOINT_ID_WRIST = 0
JOINT_ID_THUMB_X = 1
JOINT_ID_THUMB = 2
JOINT_ID_INDEX = 6
JOINT_ID_MIDDLE = 11
JOINT_ID_RING = 16
JOINT_ID_PINKY = 21


class VuerTransformer:
    def __init__(self):

        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        # Init poses
        self.vuer_head_mat = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 1.5], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
        self.vuer_left_wrist_mat = np.array(
            [[1, 0, 0, -0.3], [0, 1, 0, 1], [0, 0, 1, -0.2], [0, 0, 0, 1]]
        )
        self.vuer_right_wrist_mat = np.array(
            [[1, 0, 0, 0.3], [0, 1, 0, 1], [0, 0, 1, -0.2], [0, 0, 0, 1]]
        )

        # Transform constants

        self.hand2gripper_left = np.array(
            [[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
        )
        self.hand2gripper_right = np.array(
            [[0, 0, -1, 0], [0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]
        )

        self.grd_yup2grd_zup = np.array(
            [
                [1, 0, 0, 0],  # Map X (VR) -> X (Robot)
                [0, 0, -1, 0],  # Map Z (VR) -> Y (Robot)
                [0, 1, 0, 0],  # Map Y (VR) -> Z (Robot)
                [0, 0, 0, 1],
            ]
        )

        self.torso2head = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0.25],
                [0, 0, 1, 0.374605],
                [0, 0, 0, 1],
            ]
        )

        # Dictionary
        self.hand_joints = {
            "thumb_x": JOINT_ID_THUMB_X,
            "thumb": JOINT_ID_THUMB,
            "index": JOINT_ID_INDEX,
            "middle": JOINT_ID_MIDDLE,
            "ring": JOINT_ID_RING,
            "pinky": JOINT_ID_PINKY,
        }

    def mat_update(self, prev_mat, mat):
        if np.linalg.det(mat) == 0:
            return prev_mat
        else:
            return mat

    def fast_mat_inv(self, mat):
        ret = np.eye(4)
        ret[:3, :3] = mat[:3, :3].T
        ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
        return ret

    def transform_grd_yup2grd_zup(self, matrix):
        return self.grd_yup2grd_zup @ matrix @ self.fast_mat_inv(self.grd_yup2grd_zup)

    def translate_vr2robot(self, matrix, haed_matrix):

        rel_matrix = matrix

        # Relative to head z-axis
        rel_matrix[2, 3] = rel_matrix[2, 3] - haed_matrix[2, 3]
        # Move origin of tracking to head
        rel_matrix[0:3, 3] = rel_matrix[0:3, 3] + self.torso2head[0:3, 3]

        return rel_matrix

    def rotation_angle_3d(self, T1, T2):
        """
        Computes the rotation difference between T1 and T2.

        Args:
            T1: A 4x4 transformation matrix (reference frame).
            T2: A 4x4 transformation matrix (rotated frame).

        Returns:
            The angle of rotation in radians.
        """
        # Extract rotation matrices
        R1 = T1[:3, :3]
        R2 = T2[:3, :3]

        # Compute relative rotation
        R_relative = R2 @ R1.T

        # Compute the angle (in radians)
        trace = np.trace(R_relative)
        angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
        return angle  # In radians

    def rotation_difference_axis(self, T1, T2, axis="x"):
        """
        Computes the rotation difference around a specified axis of T1.

        Args:
            T1: A 4x4 transformation matrix (reference frame).
            T2: A 4x4 transformation matrix (rotated frame).
            axis: A string specifying the axis ("x", "y", or "z").

        Returns:
            The angle of rotation around the specified axis of T1 in radians.
        """
        # Extract rotation matrices
        R1 = T1[:3, :3]
        R2 = T2[:3, :3]

        # Compute relative rotation
        R_relative = R2 @ R1.T

        # Select the axis
        axis_dict = {"x": 0, "y": 1, "z": 2}
        if axis not in axis_dict:
            raise ValueError("Invalid axis. Choose 'x', 'y', or 'z'.")

        # Get the corresponding axis vector from T1
        axis_index = axis_dict[axis]
        axis_vector = R1[:, axis_index]

        # Transform the axis vector using the relative rotation
        transformed_axis = R_relative @ axis_vector

        # Compute the angle (in radians)
        cos_theta = np.dot(axis_vector, transformed_axis) / (
            np.linalg.norm(axis_vector) * np.linalg.norm(transformed_axis)
        )
        theta = np.arccos(np.clip(cos_theta, -1, 1))

        # Determine sign using the cross product
        cross = np.cross(axis_vector, transformed_axis)
        if cross[axis_index] < 0:  # Adjust sign based on the selected axis
            theta = -theta

        return theta  # In radians

    def process(self, vuer_app):

        # Check valid matrix
        self.vuer_head_mat = self.mat_update(
            self.vuer_head_mat, vuer_app.head_matrix.copy()
        )
        self.vuer_right_wrist_mat = self.mat_update(
            self.vuer_right_wrist_mat, vuer_app.hand_right[JOINT_ID_WRIST].copy()
        )
        self.vuer_left_wrist_mat = self.mat_update(
            self.vuer_left_wrist_mat, vuer_app.hand_left[JOINT_ID_WRIST].copy()
        )

        # change of basis
        head_mat = self.transform_grd_yup2grd_zup(self.vuer_head_mat)
        left_wrist_mat = self.transform_grd_yup2grd_zup(self.vuer_left_wrist_mat)
        right_wrist_mat = self.transform_grd_yup2grd_zup(self.vuer_right_wrist_mat)

        # Translation
        rel_left_wrist_mat = self.translate_vr2robot(left_wrist_mat, head_mat)
        rel_right_wrist_mat = self.translate_vr2robot(right_wrist_mat, head_mat)

        # Rotation
        rel_left_wrist_mat = rel_left_wrist_mat @ self.hand2gripper_left
        rel_right_wrist_mat = rel_right_wrist_mat @ self.hand2gripper_right

        # Hand/finger joints
        hand_joint_angles = {}
        for joint_name, idx in self.hand_joints.items():

            vuer_left_mat = vuer_app.hand_left[idx].copy()
            vuer_right_mat = vuer_app.hand_right[idx].copy()

            left_mat = self.transform_grd_yup2grd_zup(vuer_left_mat)
            right_mat = self.transform_grd_yup2grd_zup(vuer_right_mat)

            rel_left_mat = self.translate_vr2robot(left_mat, head_mat)
            rel_right_mat = self.translate_vr2robot(right_mat, head_mat)

            rel_left_mat = rel_left_mat @ self.hand2gripper_left
            rel_right_mat = rel_right_mat @ self.hand2gripper_left

            angle_left = np.rad2deg(
                self.rotation_difference_axis(rel_left_wrist_mat, rel_left_mat, "y")
            )
            angle_right = np.rad2deg(
                self.rotation_difference_axis(rel_right_wrist_mat, rel_right_mat, "y")
            )

            hand_joint_angles["hand_left_" + joint_name] = angle_left
            hand_joint_angles["hand_right_" + joint_name] = angle_right

        return (head_mat, rel_left_wrist_mat, rel_right_wrist_mat, hand_joint_angles)
