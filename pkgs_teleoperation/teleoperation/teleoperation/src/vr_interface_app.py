from abc import ABC
from multiprocessing import Array
import numpy as np
import logging
import os
import signal


NR_OF_HAND_JOINTS = 25
TF_MATRIX_SIZE = 16


class VRInterfaceApp(ABC):
    """
    Base class for VR interface apps for teleoperation.
    Subclasses must implement abstract methods to handle specific app functionality.
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

        # Handle termination signals
        signal.signal(signal.SIGINT, self.cleanup)
        signal.signal(signal.SIGTERM, self.cleanup)

    def __del__(self):
        """Ensure cleanup on object deletion"""
        self.cleanup()

    def cleanup(self, signum=None, frame=None):
        """Clean up resources and terminate process."""
        self.logger.info(f"Cleaning up...")

        # Terminate the process if it's still running
        if hasattr(self, "process") and self.process.is_alive():
            self.logger.info("Terminating subprocess...")
            self.process.terminate()
            self.process.join()

        self.logger.info("Cleanup complete. Exiting.")

        # Explicitly exit when cleaning up from a signal
        if signum is not None:
            os._exit(0)

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
