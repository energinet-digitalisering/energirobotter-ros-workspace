"""
Servo driver/manager of Elrik fingers of the left hand, which are servos controlled by a PCA9685. 
"""

from adafruit_pca9685 import PCA9685
import board
import busio

from servo_control.src.elrik_driver_hand import ElrikDriverHand


class ElrikDriverHandLeft(ElrikDriverHand):

    def setup_driver(self):

        self.logger.info("Initializing I2C communication with left hand PCA9685...")

        try:
            i2c_left = busio.I2C(board.SCL_1, board.SDA_1)
            pca_left = PCA9685(i2c_left, address=0x40)
            pca_left.frequency = 50

            self.logger.info("Left hand I2C communication succesful")
            return pca_left

        except Exception as e:
            self.logger.error(f"Failed to open left hand port: {e}")
            return None
