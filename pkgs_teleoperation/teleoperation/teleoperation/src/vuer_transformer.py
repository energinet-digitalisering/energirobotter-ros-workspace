import numpy as np

JOINT_ID_WRIST = 0


class VuerTransformer:
    def __init__(self):

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

    def process(self, vuer_app):
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

        return head_mat, rel_left_wrist_mat, rel_right_wrist_mat
