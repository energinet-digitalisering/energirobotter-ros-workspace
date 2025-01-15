import numpy as np


class VuerCollisionAvoidance:
    def __init__(self, safety_threshold=0.3):

        self.safety_threshold = safety_threshold

        # Init poses
        self.left_mat_prev = np.array(
            [[1, 0, 0, -0.3], [0, 1, 0, 1], [0, 0, 1, -0.2], [0, 0, 0, 1]]
        )
        self.right_mat_prev = np.array(
            [[1, 0, 0, 0.3], [0, 1, 0, 1], [0, 0, 1, -0.2], [0, 0, 0, 1]]
        )

    def mat_update(self, prev_mat, mat):
        if np.linalg.det(mat) == 0:
            return prev_mat
        else:
            return mat

    def process(self, vuer_left_mat, vuer_right_mat):

        # Check valid matrix
        left_mat = self.mat_update(self.left_mat_prev, vuer_left_mat.copy())
        right_mat = self.mat_update(self.right_mat_prev, vuer_right_mat.copy())

        left_pos = left_mat[0:3, 3]
        right_pos = right_mat[0:3, 3]

        distance = np.linalg.norm(left_pos - right_pos)

        if distance < self.safety_threshold:
            left_mat = self.left_mat_prev
            right_mat = self.right_mat_prev
        else:
            self.left_mat_prev = left_mat
            self.right_mat_prev = right_mat

        return left_mat, right_mat
