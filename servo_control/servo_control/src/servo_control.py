import numpy as np

from .servo_coms import ServoComs
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
        speed_max,  # angles/scond
        servo_id=0,
        dir=1,  # Direction config for upside-down placement (-1 or 1)
        gain_P=1.0,
        gain_I=0.0,
        gain_D=0.0,
        driver_device="arduino",
        port="/dev/ttyACM0",
    ):

        self.angle_min = angle_min
        self.angle_software_min = angle_software_min
        self.angle_max = angle_max
        self.angle_software_max = angle_software_max
        self.speed_max = speed_max
        self.dir = dir
        self.gain_P = gain_P
        self.gain_I = gain_I
        self.gain_D = gain_D

        self.angle_init = (self.angle_max / 2) + self.angle_min
        self.angle = self.angle_init

        self.error_acc = 0.0
        self.error_prev = 0.0

        self.servo_coms = ServoComs(
            pwm_min, pwm_max, angle_min, angle_max, speed_max, servo_id
        )

        self.coms_successful = False

        match driver_device:
            case "arduino":
                self.coms_successful = self.servo_coms.init_arduino(
                    port=port, baudrate=115200, timeout=1.0
                )

            case "pca9685":
                self.coms_successful = self.servo_coms.init_PCA9685()

            case _:
                print("Invalid driver device")

        self.servo_coms.write_angle(self.angle)

    def ready(self):
        return self.coms_successful

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
        self.angle += vel_control * t_d

        # Clamp values between min and max angle
        self.angle = np.clip(
            self.angle,
            self.angle_software_min,
            self.angle_software_max,
        )

        # Flip angle if direction is flipped
        if self.dir >= 0:
            angle = self.angle
        elif self.dir < 0:
            angle = interval_map(
                self.angle,
                self.angle_min,
                self.angle_max,
                self.angle_max,
                self.angle_min,
            )

        self.servo_coms.write_angle(angle)

    def reach_angle(self, t_d, angle, speed_desired=(-1)):
        angle_gain_p = 10.0
        error = (angle - self.angle) * angle_gain_p
        self.compute_control(t_d, error, speed_desired)

    def reset_position(self, t_d):
        self.reach_angle(t_d, self.angle_init)
