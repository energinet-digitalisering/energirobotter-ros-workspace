import json
import logging
import threading
from concurrent.futures import ThreadPoolExecutor
import numpy as np

from .SCServo_Python.scservo_sdk import PortHandler, sms_sts, scservo_def
from .utils import interval_map
from servo_control.src.servo_control import ServoControl

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200


class ElrikServoManager:

    def __init__(self, config_folder_path, control_frequency):

        self.logger = logging.getLogger("ElrikServoManager")
        logging.basicConfig(level=logging.INFO)

        self.config_folder_path = config_folder_path
        self.control_frequency = control_frequency

        self.servos = {}
        self.coms_active = False
        self.lock = threading.Lock()  # For thread-safe communication

        # Driver Setup
        self.logger.info("Initializing serial communication with Waveshare...")
        try:
            self.port_handler = PortHandler(PORT)
            self.packet_handler = sms_sts(self.port_handler)

            if not self.port_handler.openPort():
                self.logger.error("Failed to open port")
                self.coms_active = False
            if not self.port_handler.setBaudRate(BAUDRATE):
                self.logger.error("Failed to set baud rate")
                self.coms_active = False

            self.logger.info("Serial communication successful")
            self.coms_active = True

        except Exception as e:
            self.logger.error(f"Failed to open port: {e}")
            self.coms_active = False

        # Load and process each JSON file
        json_files = [  # List of JSON configuration files
            "servo_arm_left_nowrist_params.json",
            "servo_arm_right_nowrist_params.json",
            # "servo_arm_left_params.json",
            # "servo_arm_right_params.json",
            # "servo_right_elbow_test.json",
            # "servo_test.json",
        ]

        for json_file in json_files:
            with open(f"{config_folder_path}/{json_file}", "r") as file:
                servo_config = json.load(file)
                self._add_servos(servo_config)

    def get_default_servo_commands(self):
        command_dict = {}

        for name, servo in self.servos.items():
            command_dict[name] = 0.0

        return command_dict

    def command_servos(self, command_dict: dict):
        if not self.coms_active:
            return

        with ThreadPoolExecutor() as executor:
            futures = [
                executor.submit(self._command_servo, name, command_dict[name])
                for name in self.servos.keys()
                if name in command_dict
            ]
            for future in futures:
                try:
                    future.result()
                except Exception as e:
                    self.logger.error(f"Error commanding servo: {e}")

    def _command_servo(self, name, command):
        servo = self.servos[name]
        angle_target = command + servo.default_position
        update_flag = servo.angle != angle_target

        self.logger.error(
            f"Command: {command}, angle target: {angle_target}, current angle: {servo.angle}"
        )

        angle_cmd, pwm_cmd = servo.reach_angle(self.control_frequency, angle_target)

        if update_flag:
            self._send_command(servo, pwm_cmd)

    def update_feedback(self):
        if not self.coms_active:
            return

        with ThreadPoolExecutor() as executor:
            futures = [
                executor.submit(self._update_servo_feedback, name)
                for name in self.servos.keys()
                if self.servos[name].feedback_enabled
            ]
            for future in futures:
                try:
                    future.result()
                except Exception as e:
                    self.logger.error(f"Error updating feedback: {e}")

    def _update_servo_feedback(self, name):
        servo = self.servos[name]
        with self.lock:  # Ensure thread-safe communication
            feedback_pwm = self.packet_handler.ReadPos(servo.servo_id)[0]
        servo.set_feedback_pwm(feedback_pwm)

    def _add_servos(self, servo_config):
        """Helper method to add servos from a configuration dict."""

        for name, parameters in servo_config.items():
            self.servos[name] = ServoControl(
                servo_id=parameters["servo_id"],
                dir=parameters["dir"],
                gear_ratio=parameters["gear_ratio"],
                pwm_min=parameters["pwm_min"],
                pwm_max=parameters["pwm_max"],
                angle_min=parameters["angle_min"],
                angle_max=parameters["angle_max"],
                angle_software_min=parameters["angle_software_min"],
                angle_software_max=parameters["angle_software_max"],
                angle_speed_max=parameters["angle_speed_max"],
                default_position=parameters["default_position"],
                feedback_enabled=parameters["feedback_enabled"],
                gain_P=parameters["gain_P"],
                gain_I=parameters["gain_I"],
                gain_D=parameters["gain_D"],
            )

            self.logger.info(f"Added servo: {name}")

    def _send_command(self, servo, pwm):
        if not self.coms_active:
            return

        # Validate PWM
        pwm_min = 0
        pwm_max = 4095
        if (
            pwm is None
            or not isinstance(pwm, (int, float))
            or not (pwm_min <= pwm <= pwm_max)
        ):
            self.logger.warning(f"Invalid PWM: {pwm}")
            return

        # Validate angle
        angle = servo.pwm_2_angle(pwm)
        angle = servo.gearing_in(angle, servo.gear_ratio)

        # Flip angle if direction is flipped
        if servo.dir < 0:
            angle = interval_map(
                angle,
                servo.angle_min,
                servo.angle_max,
                servo.angle_max,
                servo.angle_min,
            )

        if angle < servo.angle_software_min:
            self.logger.warning(
                f"Servo {servo.servo_id} - Stopping pwm of {pwm}, that would result in angle of {angle}, which is below limit of {servo.angle_software_min}"
            )
            return

        if angle > servo.angle_software_max:
            self.logger.warning(
                f"Servo {servo.servo_id} - Stopping pwm of {pwm}, that would result in angle of {angle}, which is beyond limit of {servo.angle_software_max}"
            )
            return

        # Thread-safe communication
        with self.lock:
            # self.logger.info(
            #     f"Servo: {servo.servo_id}. Stopping pwm of: {pwm}, angle of: {int(np.round(angle))}"
            # )
            # return

            scs_comm_result, scs_error = self.packet_handler.WritePosEx(
                servo.servo_id, pwm, SCS_MOVING_SPEED := 1000, SCS_MOVING_ACC := 255
            )
            if scs_comm_result != scservo_def.COMM_SUCCESS:
                self.logger.error(f"Communication error: {scs_comm_result}")
            elif scs_error != 0:
                self.logger.error(f"Servo error: {scs_error}")
