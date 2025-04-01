"""
Servo driver/manager of Elrik fingers of the right hand, which are servos controlled by a PCA9685. 
"""

from adafruit_pca9685 import PCA9685
import board
import busio

from servo_control.src.elrik_driver_hand import ElrikDriverHand


class ElrikDriverHandRight(ElrikDriverHand):

    def setup_driver(self):

        self.logger.info("Initializing I2C communication with right hand PCA9685...")

        try:
            i2c_right = busio.I2C(board.SCL, board.SDA)
            pca_right = PCA9685(i2c_right, address=0x40)
            pca_right.frequency = 50

            self.logger.info("right hand I2C communication succesful")
            return pca_right

        except Exception as e:
            self.logger.error(f"Failed to open right hand port: {e}")
            return None
