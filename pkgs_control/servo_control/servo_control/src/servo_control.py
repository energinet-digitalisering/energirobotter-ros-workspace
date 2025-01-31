"""
Class for storing attributes and managing control of a single servo.
"""

import logging
import numpy as np

from .utils import interval_map


class ServoControl:
    """
    Represents a servo motor and provides methods for controlling its position and speed.

    This class supports PID control, speed limiting, and angle-to-PWM conversion.
    It also accounts for gear ratios and direction configurations.
    """

    def __init__(
        self,
        servo_id,
        pwm_min,
        pwm_max,
        angle_min,
        angle_software_min,
        angle_max,
        angle_software_max,
        angle_speed_max,
        default_position=180,
        dir=1,
        gear_ratio=1,
        gain_P=1.0,
        gain_I=0.0,
        gain_D=0.0,
        feedback_enabled=False,
    ):
        """
        Initializes the servo with given parameters.

        Args:
            servo_id (int): Unique identifier for the servo.
            pwm_min (int): Minimum PWM value.
            pwm_max (int): Maximum PWM value.
            angle_min (float): Minimum physical angle of the servo.
            angle_software_min (float): Minimum software-limited angle.
            angle_max (float): Maximum physical angle of the servo.
            angle_software_max (float): Maximum software-limited angle.
            angle_speed_max (float): Maximum angular speed (degrees/second).
            default_position (float, optional): Default angle position. Defaults to 180.
            dir (int, optional): Direction configuration for upside-down placement (1 or -1). Defaults to 1.
            gear_ratio (float, optional): Gear ratio for linked mechanisms. Defaults to 1.
            gain_P (float, optional): Proportional gain for PID control. Defaults to 1.0.
            gain_I (float, optional): Integral gain for PID control. Defaults to 0.0.
            gain_D (float, optional): Derivative gain for PID control. Defaults to 0.0.
            feedback_enabled (bool, optional): Whether feedback control is enabled. Defaults to False.
        """
        self.logger = logging.getLogger("ServoControl")
        logging.basicConfig(level=logging.INFO)

        self.servo_id = servo_id
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.angle_min = angle_min
        self.angle_software_min = angle_software_min
        self.angle_max = angle_max
        self.angle_software_max = angle_software_max
        self.angle_speed_max = angle_speed_max
        self.default_position = default_position
        self.dir = dir
        self.gear_ratio = gear_ratio
        self.gain_P = gain_P
        self.gain_I = gain_I
        self.gain_D = gain_D
        self.feedback_enabled = feedback_enabled

        self.zero_position = (angle_max + angle_min) // 2

        self.angle_init = self.default_position
        self.angle = self.angle_init
        self.pwm = self.angle_2_pwm(self.angle)

        self.error_acc = 0.0
        self.error_prev = 0.0

    def set_feedback_angle(self, feedback_angle):
        """
        Updates the servo's angle using external feedback.

        Args:
            feedback_angle (float): The measured angle from the servo's feedback system.
        """
        if not self.feedback_enabled:
            return

        self.angle = self.gearing_in(feedback_angle, self.gear_ratio)
        self.pwm = self.angle_2_pwm(feedback_angle)

    def set_feedback_pwm(self, feedback_pwm):
        """
        Updates the servo's PWM value using external feedback.

        Args:
            feedback_pwm (int): The measured PWM signal from the servo's feedback system.
        """
        if not self.feedback_enabled:
            return

        self.pwm = feedback_pwm
        self.angle = self.gearing_in(self.pwm_2_angle(feedback_pwm), self.gear_ratio)

    def controller_PID(self, error, error_acc, error_prev, gain_P, gain_I, gain_D):
        """
        Computes the PID control output based on the given error values.

        Args:
            error (float): Current error.
            error_acc (float): Accumulated error for integral control.
            error_prev (float): Previous error for derivative control.
            gain_P (float): Proportional gain.
            gain_I (float): Integral gain.
            gain_D (float): Derivative gain.

        Returns:
            float: PID control output.
        """
        kP = gain_P * error
        kI = gain_I * error_acc
        kD = gain_D * (error - error_prev)

        return kP + kI + kD

    def limit_speed(self, speed, speed_max):
        """
        Limits the speed within defined constraints.

        Args:
            speed (float): Desired speed.
            speed_max (float): Maximum allowed speed.

        Returns:
            float: Clamped speed value.
        """
        # Clamp values between desired min and max speed
        speed = np.clip(speed, -speed_max, speed_max)
        # Respect each servos individual speed limit
        speed = np.clip(speed, -self.angle_speed_max, self.angle_speed_max)

        return speed

    def compute_command(self, angle, speed_max=None):
        """
        Computes the PWM command for the servo based on the desired angle.

        Args:
            angle (float): Target angle.
            speed_max (float, optional): Maximum allowed speed. Defaults to None.

        Returns:
            tuple: (angle_cmd, pwm_cmd) as integers.
        """
        angle_cmd = angle

        if speed_max:
            speed = self.limit_speed(angle - self.angle, speed_max)
            angle_cmd = self.angle + speed

        angle_cmd = np.clip(angle_cmd, self.angle_software_min, self.angle_software_max)

        pwm_cmd = self.angle_2_pwm(angle_cmd)

        if not self.feedback_enabled:
            self.angle = angle_cmd
            self.pwm = pwm_cmd

        # Flip angle and pwm to send if direction is flipped
        if self.dir < 0:
            angle_cmd = interval_map(
                angle_cmd,
                self.angle_software_min,
                self.angle_software_max,
                self.angle_software_max,
                self.angle_software_min,
            )
            pwm_cmd = self.angle_2_pwm(angle_cmd)

        # Apply gearing ratio
        angle_cmd_geared = self.gearing_out(angle_cmd, self.gear_ratio)
        pwm_cmd_geared = self.angle_2_pwm(angle_cmd_geared)

        return int(angle_cmd_geared), int(pwm_cmd_geared)

    def compute_control(self, t_d, error, speed_max=None):

        # Compute PID control
        self.error_acc += error
        self.error_acc = np.clip(self.error_acc, -1000, 1000)  # Anti-windup

        vel_control = self.controller_PID(
            error,
            self.error_acc,
            self.error_prev,
            self.gain_P,
            self.gain_I,
            self.gain_D,
        )
        self.error_prev = error

        # Process desired speed
        speed_max = self.angle_speed_max if speed_max == None else speed_max
        angle_speed_max = (
            speed_max if speed_max < self.angle_speed_max else self.angle_speed_max
        )
        # Clamp values between min and max speed
        vel_control = self.limit_speed(vel_control, angle_speed_max)

        # Apply control to angle position
        angle_cmd = self.angle
        angle_cmd += vel_control * t_d

        return self.compute_command(angle_cmd)

    def angle_2_pwm(self, angle):
        """
        Converts an angle to a corresponding PWM value.

        Args:
            angle (float): Servo angle.

        Returns:
            int: Corresponding PWM value.
        """
        return int(
            interval_map(
                angle, self.angle_min, self.angle_max, self.pwm_min, self.pwm_max
            )
        )

    def pwm_2_angle(self, pwm):
        """
        Converts a PWM value to its corresponding angle.

        Args:
            pwm (int): PWM signal.

        Returns:
            int: Corresponding angle.
        """
        return int(
            interval_map(
                pwm, self.pwm_min, self.pwm_max, self.angle_min, self.angle_max
            )
        )

    def gearing_in(self, value, gear_ratio):
        """
        Computes the input angle of a gear based on the output angle.

        Args:
            value (float): Output angle.
            gear_ratio (float): Gear ratio.

        Returns:
            float: Input angle.
        """
        return (value - self.zero_position) / gear_ratio + self.zero_position

    def gearing_out(self, value, gear_ratio):
        """
        Computes the output angle of a gear based on the input angle.

        Args:
            value (float): Input angle.
            gear_ratio (float): Gear ratio.

        Returns:
            float: Output angle.
        """
        return self.zero_position + (value - self.zero_position) * gear_ratio

    def reach_angle_direct(self, angle, speed=None):
        """
        Moves the servo to a specific angle directly.

        Args:
            angle (float): Target angle.
            speed (float, optional): Speed limit. Defaults to None.

        Returns:
            tuple: (angle_cmd, pwm_cmd)
        """
        return self.compute_command(angle, speed)

    def reach_angle(self, t_d, angle, speed=None):
        """
        Moves the servo to a specific angle with controlled motion.

        Args:
            t_d (float): Time step.
            angle (float): Target angle.
            speed (float, optional): Speed limit. Defaults to None.

        Returns:
            tuple: (angle_cmd, pwm_cmd)
        """
        error = angle - self.angle
        return self.compute_control(t_d, error, speed)

    def reset_position(self, t_d):
        """
        Resets the servo to its initial position.

        Args:
            t_d (float): Time step.

        Returns:
            tuple: (angle_cmd, pwm_cmd)
        """
        return self.reach_angle(t_d, self.angle_init)
