import json
import logging
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

            self.logger.info("Serial communication succesful")
            self.coms_active = True

        except:
            self.logger.error("Failed to open port")
            self.coms_active = False

        # Load and process each JSON file
        json_files = [  # List of JSON configuration files
            # "servo_arm_left_params.json",
            # "servo_arm_right_params.json",
            "servo_right_elbow_test.json",
            # "servo_test.json",
        ]

        for json_file in json_files:
            with open(f"{config_folder_path}/{json_file}", "r") as file:
                servo_config = json.load(file)
                self._add_servos(servo_config)

    def command_servos(self, command_dict: dict):
        if not self.coms_active:
            return

        for name in self.servos.keys():

            angle_target = command_dict[name] + self.servos[name].default_position
            update_flag = self.servos[name].angle != angle_target

            angle_cmd, pwm_cmd = self.servos[name].reach_angle(
                self.control_frequency, angle_target
            )

            if update_flag:
                self._send_command(self.servos[name], pwm_cmd)

    def update_feedback(self):
        if not self.coms_active:
            return

        for name in self.servos.keys():
            if not self.servos[name].feedback_enabled:
                continue

            feedback_pwm = self.packet_handler.ReadPos(self.servos[name].servo_id)[0]
            self.servos[name].set_feedback_pwm(feedback_pwm)

    def get_default_servo_commands(self):
        command_dict = {}

        for name, servo in self.servos.items():
            command_dict[name] = servo.default_position

        return command_dict

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

        # Extra safety check
        angle = servo.pwm_2_angle(pwm)

        # Apply gear ratio
        angle = servo.gearing_in(angle, servo.default_position, servo.gear_ratio)

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

        # self.logger.info(f"Stopping pwm of: {pwm}, angle of: {angle}")
        # return

        scs_comm_result, scs_error = self.packet_handler.WritePosEx(
            servo.servo_id, pwm, SCS_MOVING_SPEED := 1000, SCS_MOVING_ACC := 255
        )

        # if scs_comm_result != scservo_def.COMM_SUCCESS:
        #     self.get_logger().error(
        #         f"Error in servo communication for servo id {msg.servo_id}: {scs_comm_result}"
        #     )

        # if scs_comm_result != scservo_def.COMM_SUCCESS:
        #     print("%s" % self.packet_handler.getTxRxResult(scs_comm_result))
        # elif scs_error != 0:
        #     print("%s" % self.packet_handler.getRxPacketError(scs_error))
