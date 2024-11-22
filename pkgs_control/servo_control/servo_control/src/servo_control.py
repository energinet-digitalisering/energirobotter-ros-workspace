import numpy as np

from .utils import interval_map


class ServoControl:

    def __init__(
        self,
        pwm_min,
        pwm_max,
        angle_min,
        angle_software_min,
        angle_max,
        angle_software_max,
        speed_max,  # angles/second
        dir=1,  # Direction config for upside-down placement (-1 or 1)
        gain_P=1.0,
        gain_I=0.0,
        gain_D=0.0,
    ):

        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.angle_min = angle_min
        self.angle_software_min = angle_software_min
        self.angle_max = angle_max
        self.angle_software_max = angle_software_max
        self.speed_max = speed_max
        self.dir = dir
        self.gain_P = gain_P
        self.gain_I = gain_I
        self.gain_D = gain_D
        self.feedback_enabled = False

        self.angle_init = (self.angle_max / 2) + self.angle_min
        self.angle = self.angle_init
        self.pwm = self.angle_2_pwm(self.angle)

        self.error_acc = 0.0
        self.error_prev = 0.0

    def set_feedback_angle(self, feedback_angle):
        self.angle = feedback_angle
        self.pwm = self.angle_2_pwm(feedback_angle)

    def set_feedback_pwm(self, feedback_pwm):
        self.pwm = feedback_pwm
        self.angle = self.pwm_2_angle(feedback_pwm)

    def controller_PID(self, error, error_acc, error_prev, gain_P, gain_I, gain_D):

        kP = gain_P * error
        kI = gain_I * (error_acc)
        kD = gain_D * (error - error_prev)

        return kP + kI + kD

    def compute_control(self, t_d, error, speed_desired=(-1)):

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
        speed_desired = self.speed_max if speed_desired == (-1) else speed_desired
        speed_max = speed_desired if speed_desired < self.speed_max else self.speed_max

        # Clamp values between min and max speed
        vel_control = np.clip(
            vel_control,
            speed_max * (-1),
            speed_max,
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

        return int(angle), int(pwm)

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

    def reach_angle(self, t_d, angle, speed_desired=(-1)):
        angle_gain_p = 10.0
        error = (angle - self.angle) * angle_gain_p
        return self.compute_control(t_d, error, speed_desired)

    def reset_position(self, t_d):
        return self.reach_angle(t_d, self.angle_init)
