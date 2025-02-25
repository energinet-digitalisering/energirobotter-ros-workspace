"""
Servo driver/manager of Elrik fingers, which are servos controlled by a PCA9685. 
"""

from adafruit_pca9685 import PCA9685
import board

from .utils import interval_map
from servo_control.src.elrik_driver_servos import ElrikDriverServos
from servo_control.src.servo_control import ServoControl


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

        offset = servo.angle_software_max - servo.angle_software_min
        angle_mapped = interval_map(angle_cmd, 0, 90, 0, offset)

        return angle_mapped

    def send_command(self, servo: ServoControl, pwm):

        # self.logger.info(f"Servo: {servo.servo_id}. Stopping pwm of: {pwm}")
        # return

        self.driver_object.channels[servo.servo_id].duty_cycle = pwm

    def read_feedback(self, servo: ServoControl):
        return None
