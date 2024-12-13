import logging
import numpy as np

from .utils import interval_map


class ServoControl:

    def __init__(
        self,
        servo_id,
        pwm_min,
        pwm_max,
        angle_min,
        angle_software_min,
        angle_max,
        angle_software_max,
        angle_speed_max,  # angles/second
        default_position=180,
        dir=1,  # Direction config for upside-down placement (-1 or 1)
        gear_ratio=1,
        gain_P=1.0,
        gain_I=0.0,
        gain_D=0.0,
        feedback_enabled=False,
    ):

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

        self.angle_init = self.default_position
        self.angle = self.angle_init
        self.pwm = self.angle_2_pwm(self.angle)

        self.error_acc = 0.0
        self.error_prev = 0.0

    def set_feedback_angle(self, feedback_angle):
        if not self.feedback_enabled:
            return

        self.angle = self.gearing_in(
            feedback_angle, self.default_position, self.gear_ratio
        )
        self.pwm = self.angle_2_pwm(feedback_angle)

    def set_feedback_pwm(self, feedback_pwm):
        if not self.feedback_enabled:
            return

        self.pwm = feedback_pwm
        self.angle = self.gearing_in(
            self.pwm_2_angle(feedback_pwm), self.default_position, self.gear_ratio
        )

    def controller_PID(self, error, error_acc, error_prev, gain_P, gain_I, gain_D):

        kP = gain_P * error
        kI = gain_I * (error_acc)
        kD = gain_D * (error - error_prev)

        return kP + kI + kD

    def compute_control(self, t_d, error, angle_speed_desired=(-1)):

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
        angle_speed_desired = (
            self.angle_speed_max if angle_speed_desired == (-1) else angle_speed_desired
        )
        angle_speed_max = (
            angle_speed_desired
            if angle_speed_desired < self.angle_speed_max
            else self.angle_speed_max
        )

        # angle_speed_max *= self.gear_ratio  # Increase speeds at higher gear ratios

        # Clamp values between min and max speed
        vel_control = np.clip(
            vel_control,
            angle_speed_max * (-1),
            angle_speed_max,
        )

        # Apply control to angle position
        angle = self.angle
        angle += vel_control * t_d

        # Clamp values between min and max angle
        angle = np.clip(
            angle,
            self.angle_software_min,
            self.angle_software_max,
        )
        pwm = self.angle_2_pwm(angle)

        if not self.feedback_enabled:
            self.angle = angle
            self.pwm = pwm

        # Flip angle and pwm to send if direction is flipped
        if self.dir < 0:
            angle = interval_map(
                angle,
                self.angle_min,
                self.angle_max,
                self.angle_max,
                self.angle_min,
            )
            pwm = interval_map(
                pwm,
                self.pwm_min,
                self.pwm_max,
                self.pwm_max,
                self.pwm_min,
            )

        # Apply gearing ratio
        angle_geared = self.gearing_out(angle, self.default_position, self.gear_ratio)
        pwm_geared = self.angle_2_pwm(angle_geared)

        return int(angle), int(pwm_geared)

    def angle_2_pwm(self, angle):
        pwm = int(
            interval_map(
                angle, self.angle_min, self.angle_max, self.pwm_min, self.pwm_max
            )
        )
        return pwm

    def pwm_2_angle(self, pwm):
        angle = int(
            interval_map(
                pwm, self.pwm_min, self.pwm_max, self.angle_min, self.angle_max
            )
        )
        return angle

    def gearing_in(self, value, zero_pos, gear_ratio):
        """Given the output angle of a gear, calculate the input angle"""
        offset = (value - zero_pos) / gear_ratio
        ungeared = offset + zero_pos
        return ungeared

    def gearing_out(self, value, zero_pos, gear_ratio):
        """Given the input angle of a gear, calculate the output angle"""
        offset = value - zero_pos
        geared = zero_pos + offset * gear_ratio
        return geared

    def reach_angle(self, t_d, angle, angle_speed_desired=(-1)):
        """Reach desired angle (deg)"""
        angle_gain_p = 10.0
        error = (angle - self.angle) * angle_gain_p
        return self.compute_control(t_d, error, angle_speed_desired)

    def reset_position(self, t_d):
        return self.reach_angle(t_d, self.angle_init)
