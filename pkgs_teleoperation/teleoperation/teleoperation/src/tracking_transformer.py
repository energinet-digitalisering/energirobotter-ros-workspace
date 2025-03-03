import numpy as np
import logging

# Joint identifiers
JOINT_IDS = {
    "wrist": 0,
    "thumb_x": 1,
    "thumb": 2,
    "index": 6,
    "middle": 11,
    "ring": 16,
    "pinky": 21,
}

JOINT_ROT_AXIS = {
    "wrist": "",
    "thumb_x": "x",
    "thumb": "y",
    "index": "y",
    "middle": "y",
    "ring": "y",
    "pinky": "y",
}


class TrackingTransformer:
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        # Initialize poses
        self.tracking_head_mat = self._create_matrix(translation=(0, 1.5, 0))
        self.tracking_left_wrist_mat = self._create_matrix(translation=(-0.3, 1, -0.2))
        self.tracking_right_wrist_mat = self._create_matrix(translation=(0.3, 1, -0.2))

        # # Transform constants
        self.hand2gripper_left = self._create_matrix(rotation=(0, -90, 180))
        self.hand2gripper_right = self._create_matrix(rotation=(0, 90, 180))

        self.grd_yup2grd_zup = self._create_matrix(rotation=(90, 0, 0))
        self.torso2head = self._create_matrix(translation=(0, 0.25, 0.374605))

        # Hand joints dictionary
        self.hand_joints = {k: JOINT_IDS[k] for k in JOINT_IDS if k != "wrist"}

    @staticmethod
    def _create_matrix(rotation=(0, 0, 0), translation=(0, 0, 0)):
        """
        Create a 4x4 transformation matrix from rotation (degrees) and translation.

        Args:
            rotation (tuple): Rotation angles (rx, ry, rz) in degrees.
            translation (tuple): Translation (tx, ty, tz).

        Returns:
            np.ndarray: A 4x4 transformation matrix.
        """
        rx, ry, rz = np.radians(rotation)  # Convert to radians
        tx, ty, tz = translation

        # Create individual rotation matrices
        R_x = np.array(
            [[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]]
        )

        R_y = np.array(
            [[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]]
        )

        R_z = np.array(
            [[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]]
        )

        # Combine rotations in ZYX order
        R = R_z @ R_y @ R_x

        # Construct the full transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R
        transformation_matrix[:3, 3] = [tx, ty, tz]

        return transformation_matrix

    @staticmethod
    def _update_matrix(previous, current):
        """
        Updates the matrix if the determinant of the current one is non-zero.
        """
        return current if np.linalg.det(current) != 0 else previous

    @staticmethod
    def _fast_inverse(matrix):
        """
        Computes the inverse of a 4x4 transformation matrix efficiently.
        """
        ret = np.eye(4)
        ret[:3, :3] = matrix[:3, :3].T
        ret[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
        return ret

    def _transform_basis(self, matrix):
        """
        Transforms a matrix using the GRD_YUP to GRD_ZUP convention.
        """
        return self.grd_yup2grd_zup @ matrix @ self._fast_inverse(self.grd_yup2grd_zup)

    def _translate_to_robot(self, matrix, head_matrix):
        """
        Translates a VR matrix to the robot coordinate system relative to the head.
        """
        rel_matrix = matrix.copy()
        rel_matrix[2, 3] -= head_matrix[2, 3]  # Adjust z-axis relative to the head
        rel_matrix[:3, 3] += self.torso2head[:3, 3]  # Adjust origin to head
        return rel_matrix

    def _transform_matrix(self, matrix, head_matrix, rotation):
        """
        Transforms a given matrix into the robot's coordinate system by applying
        head-relative translation and a specified rotation.

        Args:
            matrix: A 4x4 transformation matrix to be transformed.
            head_matrix: A 4x4 transformation matrix representing the head's position
                        and orientation in the VR coordinate system.
            rotation: A 4x4 rotation matrix to apply to the transformed matrix.

        Returns:
            A 4x4 transformation matrix in the robot's coordinate system.
        """
        mat_rel = (
            self._translate_to_robot(self._transform_basis(matrix), head_matrix)
            @ rotation
        )
        return mat_rel

    def _rotation_difference_axis(self, T1, T2, axis="x"):
        """
        Computes the rotation difference around a specified axis of T1.

        Args:
            T1: A 4x4 transformation matrix (reference frame).
            T2: A 4x4 transformation matrix (rotated frame).
            axis: A string specifying the axis ("x", "y", or "z").

        Returns:
            The angle of rotation around the specified axis of T1 in radians.
        """
        axis_dict = {"x": 0, "y": 1, "z": 2}
        if axis not in axis_dict:
            raise ValueError("Invalid axis. Choose 'x', 'y', or 'z'.")
        axis_index = axis_dict[axis]

        # Extract rotation matrices
        R1 = T1[:3, :3]
        R2 = T2[:3, :3]

        # Compute relative rotation
        R_relative = R2 @ R1.T

        # Get the corresponding axis vector from T1
        axis_vector = R1[:, axis_index]

        # Transform the axis vector using the relative rotation
        transformed_axis = R_relative @ axis_vector

        # Compute norms
        norm_axis = np.linalg.norm(axis_vector)
        norm_transformed = np.linalg.norm(transformed_axis)

        if norm_axis == 0 or norm_transformed == 0:
            return 0.0  # or another meaningful fallback value

        # Compute the angle (in radians)
        cos_theta = np.dot(axis_vector, transformed_axis) / (
            norm_axis * norm_transformed
        )

        theta = np.arccos(np.clip(cos_theta, -1, 1))

        # Determine sign using the cross product
        cross = np.cross(axis_vector, transformed_axis)
        if cross[axis_index] < 0:  # Adjust sign based on the selected axis
            theta = -theta

        return theta  # In radians

    def _compute_hand_angles(self, tracking_hand_left, tracking_hand_right):
        """
        Computes the joint angles for each finger.
        Tracking joint index lookup: https://docs.vuer.ai/en/latest/examples/19_hand_tracking.html
        """
        hand_angles = {}
        for joint_name, idx in self.hand_joints.items():

            # Retrieve tracking data for metacarpal and proximal finger joints
            T_metacarpal_left = tracking_hand_left[idx - 1].copy()
            T_intermediate_left = tracking_hand_left[idx + 1].copy()
            T_metacarpal_right = tracking_hand_right[idx - 1].copy()
            T_intermediate_right = tracking_hand_right[idx + 1].copy()

            # Compute relative angle between metacarpal and proximal joint
            angle_left = np.rad2deg(
                self._rotation_difference_axis(
                    T_metacarpal_left,
                    T_intermediate_left,
                    JOINT_ROT_AXIS[joint_name],
                )
            )
            angle_right = np.rad2deg(
                self._rotation_difference_axis(
                    T_metacarpal_right,
                    T_intermediate_right,
                    JOINT_ROT_AXIS[joint_name],
                )
            )

            # Cap angles between 0-90 degrees
            angle_left = np.clip(abs(angle_left), 0, 90)
            angle_right = np.clip(abs(angle_right), 0, 90)

            # Convert to rad
            angle_left = np.deg2rad(angle_left)
            angle_right = np.deg2rad(angle_right)

            # Add to dict
            hand_angles[f"hand_left_{joint_name}"] = angle_left
            hand_angles[f"hand_right_{joint_name}"] = angle_right

        return hand_angles

    def process(self, tracking_head_matrix, tracking_hand_left, tracking_hand_right):
        """
        Processes the matrices and computes the transformations and joint angles.
        """
        # Update matrices
        self.tracking_head_mat = self._update_matrix(
            self.tracking_head_mat, tracking_head_matrix.copy()
        )
        self.tracking_left_wrist_mat = self._update_matrix(
            self.tracking_left_wrist_mat, tracking_hand_left[JOINT_IDS["wrist"]].copy()
        )
        self.tracking_right_wrist_mat = self._update_matrix(
            self.tracking_right_wrist_mat,
            tracking_hand_right[JOINT_IDS["wrist"]].copy(),
        )

        # Transform
        head_matrix = self._transform_basis(self.tracking_head_mat)

        rel_left_wrist = self._transform_matrix(
            self.tracking_left_wrist_mat, head_matrix, self.hand2gripper_left
        )
        rel_right_wrist = self._transform_matrix(
            self.tracking_right_wrist_mat, head_matrix, self.hand2gripper_right
        )

        # Compute joint angles
        hand_angles = self._compute_hand_angles(tracking_hand_left, tracking_hand_right)

        return head_matrix, rel_left_wrist, rel_right_wrist, hand_angles
