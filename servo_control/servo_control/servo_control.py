import numpy as np

from .servo_coms import ServoComs


class ServoControl:

    def __init__(
        self,
        pos_min,  # angle
        pos_max,  # angle
        speed_max,  # angles/scond
        dir=1,  # Direction config for upside-down placement (-1 or 1)
        gain_P=1.0,
        gain_I=0.0,
        gain_D=0.0,
        protocol="serial",
        port="/dev/ttyACM0",
    ):

        self.pos_min = pos_min
        self.pos_max = pos_max
        self.speed_max = speed_max
        self.dir = dir
        self.gain_P = gain_P
        self.gain_I = gain_I
        self.gain_D = gain_D

        self.pos_init = (self.pos_max / 2) + self.pos_min
        self.pos = self.pos_init

        self.error_acc = 0.0
        self.error_prev = 0.0

        self.servo_coms = ServoComs()

        match protocol:
            case "serial":
                self.servo_coms.init_serial(port, baudrate=115200, timeout=1.0)

            case "i2c":
                self.servo_coms.init_i2c(port)

            case _:
                print("Invalid protocol")

        self.servo_coms.write_angle(self.pos)

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

        # Apply control to position
        self.pos += self.dir * vel_control * t_d

        # Clamp values between min and max position
        self.pos = np.clip(
            self.pos,
            self.pos_min,
            self.pos_max,
        )

        self.servo_coms.write_angle(self.pos)

    def reach_position(self, t_d, pos):
        pos_gain_p = 10.0
        error = (pos - self.pos) * pos_gain_p
        self.compute_control(error, t_d)

    def reset_position(self, t_d):
        self.reach_position(self.pos_init, t_d)
