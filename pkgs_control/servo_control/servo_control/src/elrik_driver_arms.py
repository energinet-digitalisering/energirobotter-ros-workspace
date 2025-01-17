"""
Servo driver/manager of Elrik arms, which are servos controlled by a Waveshare driver. 
"""

import numpy as np

from .SCServo_Python.scservo_sdk import PortHandler, sms_sts, scservo_def
from servo_control.src.elrik_driver_servos import ElrikDriverServos
from servo_control.src.servo_control import ServoControl

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200


class ElrikDriverArms(ElrikDriverServos):

    def setup_driver(self):

        self.logger.info("Initializing serial communication with Waveshare...")

        try:
            port_handler = PortHandler(PORT)
            packet_handler = sms_sts(port_handler)

            if not port_handler.openPort():
                self.logger.error("Failed to open port")
                return None

            if not port_handler.setBaudRate(BAUDRATE):
                self.logger.error("Failed to set baud rate")
                return None

            self.logger.info("Serial communication successful")
            return packet_handler

        except Exception as e:
            self.logger.error(f"Failed to open port: {e}")
            return None

    def send_command(self, servo: ServoControl, pwm):
        # self.logger.info(f"Servo: {servo.servo_id}. Stopping pwm of: {pwm}")
        # return

        scs_comm_result, scs_error = self.driver_object.WritePosEx(
            servo.servo_id, pwm, SCS_MOVING_SPEED := 1000, SCS_MOVING_ACC := 255
        )

        # if scs_comm_result != scservo_def.COMM_SUCCESS:
        #     self.logger.error(f"Communication error: {scs_comm_result}")
        # elif scs_error != 0:
        #     self.logger.error(f"Servo error: {scs_error}")

    def read_feedback(self, servo: ServoControl):
        self.driver_object.ReadPos(servo.servo_id)[0]
