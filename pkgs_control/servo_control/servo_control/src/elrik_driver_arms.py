"""
Servo driver/manager of Elrik arms, which are servos controlled by a Waveshare driver.
"""

import threading
import time

from .SCServo_Python.scservo_sdk import PortHandler, sms_sts, scservo_def
from servo_control.src.driver_servos import DriverServos
from servo_control.src.servo_control import ServoControl

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200


class ElrikDriverArms(DriverServos):
    def __init__(self, config_files, control_frequency):
        DriverServos.__init__(self, config_files)

        self.port_handler = None
        self.running = True
        self.lock = threading.Lock()

        self.loop_thread = threading.Thread(
            target=self.loop_send_command, args=(control_frequency,), daemon=True
        )
        self.loop_thread.start()

    def __del__(self):
        self.running = False
        self.port_handler.closePort()

    def setup_driver(self):

        self.logger.info("Initializing serial communication with Waveshare...")

        try:
            self.port_handler = PortHandler(PORT)
            packet_handler = sms_sts(self.port_handler)

            if not self.port_handler.openPort():
                self.logger.error("Failed to open port")
                return None

            if not self.port_handler.setBaudRate(BAUDRATE):
                self.logger.error("Failed to set baud rate")
                return None

            self.logger.info("Serial communication successful")
            return packet_handler

        except Exception as e:
            self.logger.error(f"Failed to open port: {e}")
            return None

    def loop_send_command(self, frequency=50):
        interval = 1.0 / frequency

        while self.running:
            start = time.time()
            self.send_command_sync()
            elapsed = time.time() - start
            time.sleep(max(0, interval - elapsed))

    def send_command_sync(self):

        with self.lock:
            # self.logger.info(f"Sync send")
            # return

            # Syncwrite goal position
            scs_comm_result = self.driver_object.groupSyncWrite.txPacket()

            if scs_comm_result != scservo_def.COMM_SUCCESS:
                self.logger.error(
                    f"Communication error: {self.driver_object.getTxRxResult(scs_comm_result)}"
                )

            # Clear syncwrite parameter storage
            self.driver_object.groupSyncWrite.clearParam()

    def send_command(self, servo: ServoControl, pwm):

        with self.lock:
            # self.logger.info(f"Servo: {servo.servo_id}. Stopping pwm of: {pwm}")
            # return

            # Add SC position\moving speed\moving accc value to the Syncwrite parameter storage
            scs_addparam_result = self.driver_object.SyncWritePosEx(
                servo.servo_id,
                pwm,
                SCS_MOVING_SPEED := 2000,
                SCS_MOVING_ACC := 64,
            )

            if scs_addparam_result != True:
                self.logger.warning(
                    f"groupSyncWrite addparam failed, servo ID: {servo.servo_id}"
                )

    def read_feedback(self, servo: ServoControl):
        try:
            return self.driver_object.ReadPos(servo.servo_id)[0]
        except Exception as e:
            self.logger.error(f"Failed to read feedback: {e}")
            return None
