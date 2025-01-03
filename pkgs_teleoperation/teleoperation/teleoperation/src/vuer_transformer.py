import numpy as np


class VuerTransformer:
    def __init__(self):
        self.vuer_head_mat = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 1.5], [0, 0, 1, -0.2], [0, 0, 0, 1]]
        )
        self.vuer_right_wrist_mat = np.array(
            [[1, 0, 0, 0.5], [0, 1, 0, 1], [0, 0, 1, -0.5], [0, 0, 0, 1]]
        )
        self.vuer_left_wrist_mat = np.array(
            [[1, 0, 0, -0.5], [0, 1, 0, 1], [0, 0, 1, -0.5], [0, 0, 0, 1]]
        )

        self.hand2gripper = np.array(
            [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
        )

        self.grd_yup2grd_zup = np.array(
            [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
        )

    def flat_to_nested_matricies(self, array, rows):
        reshaped_arrays = np.array(array).reshape((rows, 4, 4)).transpose(0, 2, 1)
        return np.array(reshaped_arrays)

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
        # self.vuer_head_mat = self.mat_update(self.vuer_head_mat, tv.head_matrix.copy())

        self.vuer_right_wrist_mat = self.mat_update(
            self.vuer_right_wrist_mat, vuer_app.hand_right[0].copy()
        )

        self.vuer_left_wrist_mat = self.mat_update(
            self.vuer_left_wrist_mat, vuer_app.hand_left[0].copy()
        )

        # change of basis

        # head_mat = (
        #     self.grd_yup2grd_zup
        #     @ self.vuer_head_mat
        #     @ self.fast_mat_inv(self.grd_yup2grd_zup)
        # )

        right_wrist_mat = (
            self.grd_yup2grd_zup
            @ self.vuer_right_wrist_mat
            @ self.fast_mat_inv(self.grd_yup2grd_zup)
        )

        left_wrist_mat = (
            self.grd_yup2grd_zup
            @ self.vuer_left_wrist_mat
            @ self.fast_mat_inv(self.grd_yup2grd_zup)
        )

        rel_left_wrist_mat = left_wrist_mat @ self.hand2gripper
        # rel_left_wrist_mat[0:3, 3] = rel_left_wrist_mat[0:3, 3] - head_mat[0:3, 3]

        rel_right_wrist_mat = right_wrist_mat @ self.hand2gripper  # wTr = wTh @ hTr
        # rel_right_wrist_mat[0:3, 3] = rel_right_wrist_mat[0:3, 3] - head_mat[0:3, 3]

        head_mat = None

        return head_mat, rel_left_wrist_mat, rel_right_wrist_mat
