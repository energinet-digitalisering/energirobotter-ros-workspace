import numpy as np

JOINT_ID_WRIST = 0


class VuerTransformer:
    def __init__(self):

        # Init poses
        self.vuer_head_mat = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 1.5], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
        self.vuer_left_wrist_mat = np.array(
            [[1, 0, 0, -0.5], [0, 1, 0, 1], [0, 0, 1, -0.5], [0, 0, 0, 1]]
        )
        self.vuer_right_wrist_mat = np.array(
            [[1, 0, 0, 0.5], [0, 1, 0, 1], [0, 0, 1, -0.5], [0, 0, 0, 1]]
        )

        # Transform constants
        self.hand2gripper = np.array(
            [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
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
                [0, 1, 0, 0.01],
                [0, 0, 1, 0.374605],
                [0, 0, 0, 1],
            ]
        )

        self.fixed_rotation_left = np.array(
            [
                [0, 0, 1],
                [0, -1, 0],
                [1, 0, 0],
            ]
        )

        self.fixed_rotation_right = np.array(
            [
                [0, 0, -1],
                [0, -1, 0],
                [-1, 0, 0],
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

        head_mat = (
            self.grd_yup2grd_zup
            @ self.vuer_head_mat
            @ self.fast_mat_inv(self.grd_yup2grd_zup)
        )

        left_wrist_mat = (
            self.grd_yup2grd_zup
            @ self.vuer_left_wrist_mat
            @ self.fast_mat_inv(self.grd_yup2grd_zup)
        )

        right_wrist_mat = (
            self.grd_yup2grd_zup
            @ self.vuer_right_wrist_mat
            @ self.fast_mat_inv(self.grd_yup2grd_zup)
        )

        rel_left_wrist_mat = left_wrist_mat
        # rel_left_wrist_mat = left_wrist_mat @ self.hand2gripper
        rel_left_wrist_mat[0:3, 3] = (
            rel_left_wrist_mat[0:3, 3] - head_mat[0:3, 3] + self.torso2head[0:3, 3]
        )

        rel_right_wrist_mat = right_wrist_mat
        # rel_right_wrist_mat = right_wrist_mat @ self.hand2gripper  # wTr = wTh @ hTr
        rel_right_wrist_mat[0:3, 3] = (
            rel_right_wrist_mat[0:3, 3] - head_mat[0:3, 3] + self.torso2head[0:3, 3]
        )

        # Override rotation matrix
        rel_left_wrist_mat[0:3, 0:3] = self.fixed_rotation_left
        rel_right_wrist_mat[0:3, 0:3] = self.fixed_rotation_right

        return head_mat, rel_left_wrist_mat, rel_right_wrist_mat
