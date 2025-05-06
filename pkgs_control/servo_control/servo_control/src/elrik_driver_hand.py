"""
Servo driver/manager of Elrik fingers, which are servos controlled by a PCA9685.
"""

from adafruit_pca9685 import PCA9685
import board

from .utils import interval_map
from servo_control.src.elrik_driver_servos import ElrikDriverServos
from servo_control.src.servo_control import ServoControl

import time
from collections import deque

call_times = deque()


class ElrikDriverHand(ElrikDriverServos):

    def setup_driver(self):

        self.logger.info("Initializing I2C communication with PCA9685...")

        try:
            i2c = board.I2C()
            pca = PCA9685(i2c)
            pca.frequency = 50

            self.logger.info("I2C communication succesful")
            return pca

        except Exception as e:
            self.logger.error(f"Failed to open port: {e}")
            return None

    @staticmethod
    def map_finger_to_servo(servo: ServoControl, angle_cmd):

        angle_mapped = interval_map(
            angle_cmd,
            0,
            90,
            servo.angle_software_min - servo.default_position,
            servo.angle_software_max - servo.default_position,
        )

        return angle_mapped

    def send_command(self, servo: ServoControl, pwm):

        now = time.time()
        call_times.append(now)

        # Remove timestamps older than 10 seconds
        while call_times and call_times[0] < now - 10:
            call_times.popleft()

        frequency = len(call_times) / 10  # Calls per second in the last 10 sec
        # self.logger.info(f"Frequency (last 10s): {frequency:.2f} calls/sec")

        # self.logger.info(f"Servo: {servo.servo_id}. Stopping pwm of: {pwm}")
        # return

        self.driver_object.channels[servo.servo_id].duty_cycle = pwm

    def read_feedback(self, servo: ServoControl):
        return None
