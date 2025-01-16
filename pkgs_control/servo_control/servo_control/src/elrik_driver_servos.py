"""
Base class for servo driver/managers of Elrik. 
"""

from abc import ABC, abstractmethod
import json
import logging
import threading
from concurrent.futures import ThreadPoolExecutor

from .utils import interval_map
from servo_control.src.servo_control import ServoControl


class ElrikDriverServos(ABC):
    """
    Base class for servo driver/managers of Elrik.
    This class provides a framework for managing and controlling multiple servos.
    Subclasses must implement abstract methods to handle specific driver functionality.
    """

    def __init__(self, config_files, control_frequency):

        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.control_frequency = control_frequency
        self.servos = {}
        self.coms_active = False
        self.lock = threading.Lock()  # For thread-safe communication

        # Load and process JSON files
        for json_file in config_files:
            with open(json_file, "r") as file:
                servo_config = json.load(file)
                self._add_servos(servo_config)

        self.driver_object = self.setup_driver()

        if self.driver_object != None:
            self.coms_active = True
        else:
            self.coms_active = False

    def _add_servos(self, servo_config):
        """
        Add servos to the driver based on the provided configuration.

        Args:
            servo_config (dict): Dictionary containing servo parameters.
        """

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

    @abstractmethod
    def setup_driver(self):
        """
        Abstract method to initialize the driver object for communication with servos.
        Must be implemented by subclasses, and return a driver_object.
        """
        pass

    @abstractmethod
    def send_command(self, servo: ServoControl, pwm):
        """
        Abstract method to send a command to a servo.

        Args:
            servo (ServoControl): Servo object containing servo attributes.
            pwm (float): PWM value to command the servo.
        """
        pass

    @abstractmethod
    def read_feedback(self):
        """
        Abstract method to read feedback from the servos.
        Must return a feedback PWM value.
        """
        pass

    def command_servos(self, command_dict: dict):
        """
        Send commands to multiple servos using a dictionary of servo names and commands.

        Args:
            command_dict (dict): Dictionary mapping servo names to desired commands.
        """
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

    def update_feedback(self):
        """
        Update feedback values for all servos with feedback enabled.
        This method fetches feedback from the hardware and updates each servo's state.
        """
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

    def get_default_servo_commands(self):
        """
        Generate a dictionary of default commands for all servos.

        Returns:
            dict: A dictionary mapping servo names to default command values (e.g., 0.0).
        """
        return {name: 0.0 for name in self.servos}

    def _command_servo(self, name, command):
        """
        Send a command to a single servo.

        Args:
            name (str): Name of the servo.
            command (float): Desired command value (e.g., target angle).
        """
        servo = self.servos[name]
        angle_target = command + servo.default_position
        update_flag = int(servo.angle) != int(angle_target)

        if not update_flag:
            return

        angle_cmd, pwm_cmd = servo.reach_angle_direct(angle_target)

        if not self._validate_command(servo, pwm_cmd):
            return

        with self.lock:
            self.send_command(servo, pwm_cmd)

    def _validate_command(self, servo: ServoControl, pwm):
        """
        Validate a command before sending it to a servo.

        Args:
            servo (ServoControl): Servo object being validated.
            pwm (float): PWM value to validate.

        Returns:
            bool: True if the command is valid, False otherwise.
        """
        if not self.coms_active:
            return False

        # Validate PWM
        if (
            pwm is None
            or not isinstance(pwm, (int, float))
            or not (servo.pwm_min <= pwm <= servo.pwm_max)
        ):
            self.logger.warning(f"Invalid PWM: {pwm}")
            return False

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

        if angle < servo.angle_software_min or angle > servo.angle_software_max:
            self.logger.debug(
                f"Servo {servo.servo_id} - PWM {pwm} would result in angle {angle}, "
                f"which is outside the software limits ({servo.angle_software_min}, {servo.angle_software_max})"
            )
            return False

        return True

    def _update_servo_feedback(self, name):
        """
        Update feedback for a single servo.

        Args:
            name (str): Name of the servo to update feedback for.
        """
        with self.lock:  # Ensure thread-safe communication
            feedback_pwm = self.read_feedback()
        self.servos[name].set_feedback_pwm(feedback_pwm)
