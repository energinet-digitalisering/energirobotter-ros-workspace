import numpy as np
from enum import Enum
import serial


class Protocol(Enum):
    UNINITIALIZED = 0
    SERIAL = 1
    I2C = 2


class ServoComs:

    def __init__(self):

        self.protocol = Protocol.UNINITIALIZED

    def __del__(self):
        if self.protocol == Protocol.SERIAL:
            print("Closing serial connection")
            self.serial.close()

    def init_serial(self, port="/dev/ttyACM0", baudrate=115200, timeout=1.0):
        try:
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            self.protocol = Protocol.SERIAL
        except:
            self.protocol = Protocol.UNINITIALIZED

    def init_i2c(self, port="/dev/ttyACM0", baudrate=115200, timeout=1.0):
        print("I2C protocol unimplemented")
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

    def write_angle_serial(self, value):

        if self.protocol == Protocol.SERIAL:
            value = int(np.round(value))

            # write packet to serial
            self.serial.write(bytes(str(int(value)), "utf-8"))
            self.serial.write(bytes("\n", "utf-8"))  # End character
        else:
            print("No serial available")
