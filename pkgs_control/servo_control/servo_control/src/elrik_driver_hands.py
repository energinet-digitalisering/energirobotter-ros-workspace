"""
Servo driver/manager of Elrik fingers, which are servos controlled by a PCA9685. 
"""

from adafruit_pca9685 import PCA9685
import board

from servo_control.src.elrik_driver_servos import ElrikDriverServos
from servo_control.src.servo_control import ServoControl


class ElrikDriverHands(ElrikDriverServos):

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

    def send_command(self, servo: ServoControl, pwm):
        self.driver_object.channels[servo.servo_id].duty_cycle = pwm

    def read_feedback(self, servo: ServoControl):
        return None
