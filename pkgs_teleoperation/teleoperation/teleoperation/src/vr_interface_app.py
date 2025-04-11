from abc import ABC
from multiprocessing import Array
import numpy as np
import logging

from teleoperation.src.tracking_constants import TF_MATRIX_SIZE, NR_OF_HAND_JOINTS


class VRInterfaceApp(ABC):
    """
    Base class for VR interface apps for teleoperation.
    Subclasses must implement methods to handle specific app functionality.
    """

    def __init__(self):
        # Initialize logging
        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.head_matrix_shared = Array("d", (TF_MATRIX_SIZE), lock=True)
        self.hand_left_shared = Array(
            "d", (NR_OF_HAND_JOINTS * TF_MATRIX_SIZE), lock=True
        )
        self.hand_right_shared = Array(
            "d", (NR_OF_HAND_JOINTS * TF_MATRIX_SIZE), lock=True
        )

    def __del__(self):
        """Ensure cleanup on object deletion"""
        self.logger.info("Cleaning up...")

        # Terminate the process if it's still running
        if hasattr(self, "process") and self.process.is_alive():
            self.logger.info("Terminating subprocess...")
            self.process.terminate()
            self.process.join()

        self.logger.info("Cleanup complete. Exiting.")

    @property
    def head_matrix(self):
        return np.array(self.head_matrix_shared).reshape((4, 4)).transpose(1, 0)

    @property
    def hand_left(self):
        return (
            np.array(self.hand_left_shared)
            .reshape((NR_OF_HAND_JOINTS, 4, 4))
            .transpose(0, 2, 1)
        )

    @property
    def hand_right(self):
        return (
            np.array(self.hand_right_shared)
            .reshape((NR_OF_HAND_JOINTS, 4, 4))
            .transpose(0, 2, 1)
        )
