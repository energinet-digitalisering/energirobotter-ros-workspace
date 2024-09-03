from adafruit_pca9685 import PCA9685
import numpy as np
import board
from enum import Enum
import serial


class Protocol(Enum):
    UNINITIALIZED = 0
    SERIAL = 1
    I2C = 2


class ServoComs:

    def __init__(self, pwm_min, pwm_max, angle_min, angle_max, servo_id=0):

        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.servo_id = servo_id

        self.protocol = Protocol.UNINITIALIZED

    def __del__(self):
        if self.protocol == Protocol.SERIAL:
            print("Closing serial connection")
            self.serial.close()

        if self.protocol == Protocol.I2C:
            self.pca.deinit()

    def init_serial(self, port="/dev/ttyACM0", baudrate=115200, timeout=1.0):
        print("Initializing serial communication...")
        try:
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            self.protocol = Protocol.SERIAL
            print("Serial communication succesful")
        except:
            self.protocol = Protocol.UNINITIALIZED
            print("Serial not available")

    def init_i2c(self):
        print("I2C protocol unimplemented")

        i2c = board.I2C()
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.protocol = Protocol.I2C

    def write_angle(self, value):
        match self.protocol:
            case Protocol.UNINITIALIZED:
                print("No protocol initialized")

            case Protocol.SERIAL:
                self.write_angle_serial(value)

            case Protocol.I2C:
                print("I2C protocol unimplemented")

            case _:
                print("Invalid protocol")

    def write_angle_serial(self, angle):

        if self.protocol == Protocol.SERIAL:
            angle = int(np.round(angle))

            # write packet to serial
            self.serial.write(bytes(str(int(angle)), "utf-8"))
            self.serial.write(bytes("\n", "utf-8"))  # End character
        else:
            print("Serial not available")

    def write_angle_i2c(self, angle):

        if self.protocol == Protocol.I2C:
            angle = int(np.round(angle))
            pwm = self.interval_map(
                angle, self.angle_min, self.angle_max, self.pwm_min, self.pwm_max
            )

            pwm_hex = hex(pwm) + "f"  # 12-bit to 16-bit
            self.pca.channels[self.servo_id].duty_cycle = pwm_hex

        else:
            print("I2C not available")

    def interval_map(self, x, x0, x1, y0, y1):

        # x: value
        # [x0, x1]: original interval
        # [y0, y1]: target interval

        return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0)
