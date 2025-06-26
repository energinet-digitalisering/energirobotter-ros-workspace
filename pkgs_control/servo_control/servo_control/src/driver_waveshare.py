"""
Servo driver/manager of humanoid robot servos, which are servos controlled by a Waveshare driver.
"""

import threading
import time

from .SCServo_Python.scservo_sdk import PortHandler, sms_sts, scservo_def
from .utils import interval_map
from servo_control.src.driver_servos import DriverServos
from servo_control.src.servo_control import ServoControl

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200


class DriverWaveshare(DriverServos):
    def __init__(self, config_files, control_frequency):
        DriverServos.__init__(self, config_files)

        self.port_handler = None
        self.running = True
        self.lock = threading.Lock()

        self.loop_thread_read = threading.Thread(
            target=self.loop_sync_commands_read, args=(control_frequency,), daemon=True
        )
        self.loop_thread_read.start()

        self.loop_thread_write = threading.Thread(
            target=self.loop_sync_commands_write, args=(control_frequency,), daemon=True
        )
        self.loop_thread_write.start()

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

    def loop_sync_commands_read(self):
        interval = 1.0

        while self.running:
            start = time.time()
            self.sync_commands_read()
            elapsed = time.time() - start
            time.sleep(max(0, interval - elapsed))

    def loop_sync_commands_write(self, frequency=10):
        interval = 1.0 / frequency

        while self.running:
            start = time.time()
            self.sync_commands_write()
            elapsed = time.time() - start
            time.sleep(max(0, interval - elapsed))

    def sync_commands_read(self):

        with self.lock:

            # Sync read
            self.driver_object.groupSyncRead.clearParam()

            for servo in self.servos.values():
                scs_addparam_result = self.driver_object.groupSyncRead.addParam(
                    servo.servo_id
                )

            scs_comm_result = self.driver_object.groupSyncRead.txRxPacket()
            # if scs_comm_result != scservo_def.COMM_SUCCESS:
            #     self.logger.error(
            #         f"Communication error while reading: {self.driver_object.getTxRxResult(scs_comm_result)}"
            #     )

    def sync_commands_write(self):

        with self.lock:

            # Sync write
            scs_comm_result = self.driver_object.groupSyncWrite.txPacket()
            # if scs_comm_result != scservo_def.COMM_SUCCESS:
            #     self.logger.error(
            #         f"Communication error while writing: {self.driver_object.getTxRxResult(scs_comm_result)}"
            #     )
            self.driver_object.groupSyncWrite.clearParam()

    def read_feedback(self, servo: ServoControl):

        try:
            # return self.driver_object.ReadPos(servo.servo_id)[0]
            # return self.driver_object.ReadTemperature(servo.servo_id)[0]

            feedback = self.driver_object.SyncRead(servo.servo_id)
            return feedback

        except Exception as e:
            self.logger.error(f"Failed to read feedback: {e}")
            return None

    def write_command(self, servo: ServoControl, pwm):

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

            # if scs_addparam_result != True:
            # self.logger.warning(
            # f"groupSyncWrite addparam failed, servo ID: {servo.servo_id}"
            # )

    def map_finger_to_servo(servo: ServoControl, angle_cmd):
        # Function specific to finger servos, that takes an angle between 0-90 and converts to correct range

        angle_mapped = interval_map(
            angle_cmd,
            0,
            90,
            servo.angle_software_min - servo.default_position,
            servo.angle_software_max - servo.default_position,
        )

        return angle_mapped
