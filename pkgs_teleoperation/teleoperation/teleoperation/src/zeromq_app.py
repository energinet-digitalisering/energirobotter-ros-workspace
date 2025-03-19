import numpy as np
from scipy.spatial.transform import Rotation

from teleoperation.src.tracking_constants import HAND_JOINT_IDS
from teleoperation.src.vr_interface_app import VRInterfaceApp


class ZeroMQApp(VRInterfaceApp):

    def lefthand2righthand(self, rotation_matrix_l):
        """
        Converts a rotation matrix from a left-handed to a right-handed coordinate system.

        Parameters:
            rotation_matrix (numpy.ndarray): A 3x3 rotation matrix in a left-handed coordinate system.

        Returns:
            numpy.ndarray: A new 3x3 rotation matrix transformed to a right-handed coordinate system.

        Note:
            - Assumes a column-major representation; for row-major, negate the third row instead.
        """
        rotation_matrix_r = rotation_matrix_l.copy()
        rotation_matrix_r[:, 2] *= -1
        return rotation_matrix_r

    def dict_to_tf_matrix(self, tf_dict):
        translation = np.array(tf_dict["position"])
        quaternion = np.array(tf_dict["rotation"])

        # Convert quaternion to rotation matrix
        rotation = Rotation.from_quat(quaternion)
        rotation_matrix = rotation.as_matrix()

        # Construct a 4x4 transformation matrix
        tf_matrix = np.eye(4)  # Start with an identity matrix

        tf_matrix[:3, 3] = translation  # Set the translation part
        tf_matrix[:3, :3] = rotation_matrix  # Assign the rotation matrix

        return tf_matrix

    def to_tf_matrix(self, translation, quaternion):
        # Convert quaternion to rotation matrix
        rotation = Rotation.from_quat(quaternion)
        rotation_matrix = rotation.as_matrix()
        rotation_matrix = self.lefthand2righthand(rotation_matrix)

        # Construct a 4x4 transformation matrix
        tf_matrix = np.eye(4)  # Start with an identity matrix

        tf_matrix[:3, 3] = translation  # Set the translation part
        # tf_matrix[2, 3] *= -1  # Flip z-axis
        tf_matrix[:3, :3] = rotation_matrix  # Assign the rotation matrix

        return tf_matrix

    def write_matrix_to_array(self, matrix, array, start_index=0):
        """
        Converts a 4x4 NumPy matrix to column-major order and writes it directly into the array
        starting from the given index.

        Parameters:
            matrix (np.ndarray): 4x4 NumPy array to be written.
            array (np.ndarray): 1D array where the matrix data should be written.
            start_index (int): Index in the array where the matrix data should start.
        """
        assert matrix.shape == (4, 4), "Input matrix must be 4x4"
        assert isinstance(matrix, np.ndarray), "Input matrix must be a NumPy array"
        # assert isinstance(array, np.ndarray), "Array must be a NumPy array"
        assert (
            array.get_obj()._length_ >= start_index + 16
        ), "Not enough space in the target array"

        # Modify the array in-place
        array[start_index : start_index + 16] = matrix.T.reshape(-1)

    def update_tracking(self, tracking_dict: dict):

        # Head
        # tf_matrix = self.dict_to_tf_matrix(tracking_dict["head"])
        # self.write_matrix_to_array(tf_matrix, self.head_matrix_shared)

        tf_matrix = self.to_tf_matrix(
            np.array(
                [
                    tracking_dict["headPos"]["x"],
                    tracking_dict["headPos"]["y"],
                    tracking_dict["headPos"]["z"],
                ]
            ),
            np.array(
                [
                    tracking_dict["headRot"]["x"],
                    tracking_dict["headRot"]["y"],
                    tracking_dict["headRot"]["z"],
                    tracking_dict["headRot"]["w"],
                ]
            ),
        )

        self.write_matrix_to_array(tf_matrix, self.head_matrix_shared)

        # # Left hand
        # for joint_name, tf_dict in tracking_dict["hand_left"].items():
        #     tf_matrix = self.dict_to_tf_matrix(tf_dict)
        #     self.write_matrix_to_array(
        #         tf_matrix, self.hand_left_shared, HAND_JOINT_IDS[joint_name]
        #     )

        tf_matrix = self.to_tf_matrix(
            np.array(
                [
                    tracking_dict["leftHandPos"]["x"],
                    tracking_dict["leftHandPos"]["y"],
                    tracking_dict["leftHandPos"]["z"],
                ]
            ),
            np.array(
                [
                    tracking_dict["leftHandRot"]["x"],
                    tracking_dict["leftHandRot"]["y"],
                    tracking_dict["leftHandRot"]["z"],
                    tracking_dict["leftHandRot"]["w"],
                ]
            ),
        )

        self.logger.info(f"tf_matrix: \n {tf_matrix}")

        self.write_matrix_to_array(
            tf_matrix, self.hand_left_shared, HAND_JOINT_IDS["wrist"]
        )

        # # Right hand
        # for joint_name, tf_dict in tracking_dict["hand_right"].items():
        #     tf_matrix = self.dict_to_tf_matrix(tf_dict)
        #     self.write_matrix_to_array(
        #         tf_matrix, self.hand_right_shared, HAND_JOINT_IDS[joint_name]
        #     )

        tf_matrix = self.to_tf_matrix(
            np.array(
                [
                    tracking_dict["rightHandPos"]["x"],
                    tracking_dict["rightHandPos"]["y"],
                    tracking_dict["rightHandPos"]["z"],
                ]
            ),
            np.array(
                [
                    tracking_dict["rightHandRot"]["x"],
                    tracking_dict["rightHandRot"]["y"],
                    tracking_dict["rightHandRot"]["z"],
                    tracking_dict["rightHandRot"]["w"],
                ]
            ),
        )

        self.write_matrix_to_array(
            tf_matrix, self.hand_right_shared, HAND_JOINT_IDS["wrist"]
        )


if __name__ == "__main__":
    zeromq_app = ZeroMQApp()
