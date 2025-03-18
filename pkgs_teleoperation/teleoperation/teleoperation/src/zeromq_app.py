import numpy as np
from scipy.spatial.transform import Rotation

from teleoperation.src.tracking_constants import HAND_JOINT_IDS
from teleoperation.src.vr_interface_app import VRInterfaceApp


class ZeroMQApp(VRInterfaceApp):

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
        assert isinstance(array, np.ndarray), "Array must be a NumPy array"
        assert array.size >= start_index + 16, "Not enough space in the target array"

        # Modify the array in-place
        array[start_index : start_index + 16] = matrix.T.reshape(-1)

    def update_tracking(self, tracking_dict: dict):

        # Head
        tf_matrix = self.dict_to_tf_matrix(tracking_dict["head"])
        self.write_matrix_to_array(tf_matrix, self.head_matrix_shared)

        # Left hand
        for joint_name, tf_dict in tracking_dict["hand_left"].items():
            tf_matrix = self.dict_to_tf_matrix(tf_dict)
            self.write_matrix_to_array(
                tf_matrix, self.hand_left_shared, HAND_JOINT_IDS[joint_name]
            )

        # Right hand
        for joint_name, tf_dict in tracking_dict["hand_right"].items():
            tf_matrix = self.dict_to_tf_matrix(tf_dict)
            self.write_matrix_to_array(
                tf_matrix, self.hand_right_shared, HAND_JOINT_IDS[joint_name]
            )


if __name__ == "__main__":
    zeromq_app = ZeroMQApp()
