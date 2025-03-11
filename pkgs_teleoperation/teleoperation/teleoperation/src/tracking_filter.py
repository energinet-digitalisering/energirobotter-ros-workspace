from collections import deque
import numpy as np
import logging
import scipy.signal


class TrackingFilter:
    def __init__(
        self,
        alpha_ema=0.2,
        alpha_low_pass=0.2,
        window_size=5,
        butter_cutoff=0.1,
        butter_order=2,
    ):
        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.alpha_ema = alpha_ema  # For exponential moving average
        self.alpha_low_pass = alpha_low_pass  # For low-pass filter
        self.window_size = window_size  # For moving average
        self.butter_cutoff = butter_cutoff  # For Butterworth filter
        self.butter_order = butter_order  # For Butterworth filter

        self.moving_avg_queue = deque(maxlen=self.window_size)
        self.data_ema = []
        self.data_low_pass = []
        self.butter_data = []
        self.b, self.a = scipy.signal.butter(
            butter_order, butter_cutoff, btype="low", analog=False
        )

    def moving_average(self, tf_matrix):
        """Applies a simple moving average filter"""
        self.moving_avg_queue.append(tf_matrix)
        return (
            np.mean(self.moving_avg_queue, axis=0)
            if len(self.moving_avg_queue) > 0
            else tf_matrix
        )

    def exp_moving_average(self, tf_matrix):
        """Applies an exponential moving average (EMA) filter"""
        if len(self.data_ema) == 0:
            self.data_ema = tf_matrix

        self.data_ema = (
            self.alpha_ema * tf_matrix + (1 - self.alpha_ema) * self.data_ema
        )
        return self.data_ema

    def low_pass(self, tf_matrix):
        """Applies a first-order low-pass filter"""
        if len(self.data_low_pass) == 0:
            self.data_low_pass = tf_matrix

        self.data_low_pass = (
            self.alpha_low_pass * tf_matrix
            + (1 - self.alpha_low_pass) * self.data_low_pass
        )
        return self.data_low_pass

    def butterworth(self, tf_matrix):
        """Applies a Butterworth low-pass filter."""
        self.butter_data.append(tf_matrix)

        if len(self.butter_data) < 10:  # Ensure enough data points
            return tf_matrix  # Return the original matrix until we have enough data

        filtered_data = scipy.signal.filtfilt(
            self.b, self.a, np.array(self.butter_data), axis=0
        )
        return filtered_data[-1]  # Return the latest filtered value

    def process(self, tracking_hand_left, tracking_hand_right):
        """
        Filters tracking transform matrices.
        """

        return self._low_pass(tracking_hand_left), self._low_pass(tracking_hand_right)
