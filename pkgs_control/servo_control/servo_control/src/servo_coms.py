from adafruit_pca9685 import PCA9685
import board
from enum import Enum
import numpy as np
import serial

from .utils import interval_map
from .SCServo_Python.scservo_sdk import PortHandler, sms_sts, scservo_def


class DriverDevice(Enum):
    UNINITIALIZED = 0
    ARDUINO = 1
    PCA9685 = 2
    WAVESHARE_DRIVER = 3


class ServoComs:

    def __init__(self, pwm_min, pwm_max, angle_min, angle_max, speed_max, servo_id=0):

        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.speed_max = speed_max
        self.servo_id = servo_id

        self.driver_device = DriverDevice.UNINITIALIZED

        self.mute_spam_print = False

        print("Created servo with id: ", servo_id)

    def __del__(self):
        if self.driver_device == DriverDevice.ARDUINO:
            print("Closing serial connection")
            self.serial.close()

        if self.driver_device == DriverDevice.PCA9685:
            self.pca.deinit()

        if self.driver_device == DriverDevice.WAVESHARE_DRIVER:
            self.port_handler.closePort()

    def init_arduino(self, port="/dev/ttyACM0", baudrate=115200, timeout=1.0):
        print("Initializing serial communication...")
        try:
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            self.driver_device = DriverDevice.ARDUINO
            print("Serial communication succesful")
            return True
        except:
            self.driver_device = DriverDevice.UNINITIALIZED
            print("Communication with Arduino unsuccesful, serial not available")
            return False

    def init_PCA9685(self):
        print("Initializing I2C communication with PCA9685...")
        try:
            i2c = board.I2C()
            self.pca = PCA9685(i2c)
            self.pca.frequency = 50
            self.driver_device = DriverDevice.PCA9685
            print("I2C communication with PCA9685 succesful")
            return True
        except:
            self.driver_device = DriverDevice.UNINITIALIZED
            print("Communication with PCA9685 unsuccesful, I2C not available")
            return False

    def init_waveshare_driver(self, port="/dev/ttyUSB0", baudrate=1000000):
        self.port_handler = PortHandler(port)
        self.packet_handler = sms_sts(self.port_handler)

        if self.port_handler.openPort():
            self.port_handler.setBaudRate(baudrate)
            self.driver_device = DriverDevice.WAVESHARE_DRIVER
            print("Serial communication with Waveshare Driver succesful")
            return True
        else:
            self.driver_device = DriverDevice.UNINITIALIZED
            print(
                "Communication with Waveshare Driver unsuccesful, serial not available"
            )
            return False

    def write_angle(self, value):
        match self.driver_device:
            case DriverDevice.UNINITIALIZED:
                if not self.mute_spam_print:
                    print("Cannot write angle to servo, no driver device initialized")
                self.mute_spam_print = True

            case DriverDevice.ARDUINO:
                self.write_angle_arduino(value)

            case DriverDevice.PCA9685:
                self.write_angle_pca9685(value)

            case DriverDevice.WAVESHARE_DRIVER:
                self.write_angle_waveshare_driver(value)

            case _:
                print("Invalid driver device")

    def write_angle_arduino(self, angle):

        if self.driver_device == DriverDevice.ARDUINO:
            angle = int(np.round(angle))

            # write packet to serial
            self.serial.write(bytes(str(int(angle)), "utf-8"))
            self.serial.write(bytes("\n", "utf-8"))  # End character
        else:
            print("Arduino not available")

    def write_angle_pca9685(self, angle):

        if self.driver_device == DriverDevice.PCA9685:
            angle = int(np.round(angle))
            pwm = int(
                interval_map(
                    angle, self.angle_min, self.angle_max, self.pwm_min, self.pwm_max
                )
            )

            self.pca.channels[self.servo_id].duty_cycle = pwm

        else:
            print("PCA9685 not available")

    def write_angle_waveshare_driver(self, angle):

        if self.driver_device == DriverDevice.WAVESHARE_DRIVER:
            angle = int(np.round(angle))

            pwm = int(
                interval_map(
                    angle, self.angle_min, self.angle_max, self.pwm_min, self.pwm_max
                )
            )

            scs_comm_result, scs_error = self.packet_handler.WritePosEx(
                self.servo_id,
                pwm,
                self.speed_max,
                SCS_MOVING_ACC := 255,  # SC Servo moving acc (in 8-bit)
            )
            if scs_comm_result != scservo_def.COMM_SUCCESS:
                print("%s" % self.packet_handler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(scs_error))

        else:
            print("Waveshare Driver not available")
