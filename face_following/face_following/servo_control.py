import numpy as np
import serial


class ServoControl:

    def __init__(
        self,
        pos_min,  # PWM
        pos_max,  # PWM
        speed_min,  # PWM/s
        speed_max,  # PWM/s
        dir=1,  # Direction config for upside-down placement (-1 or 1)
        p_gain=1.0,
    ):

        self.pos_min = pos_min
        self.pos_max = pos_max
        self.speed_min = speed_min
        self.speed_max = speed_max
        self.dir = dir
        self.p_gain = p_gain

        self.pos = self.pos_max / 2

        self.serial = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1.0)

        self.serial_write_angle(self.pos)

    def __del__(self):
        print("Closing serial connection")
        self.serial.close()

    def serial_write_angle(self, value):
        value = int(np.round(value))

        # write packet to serial
        self.serial.write(bytes(str(int(value)), "utf-8"))
        self.serial.write(bytes("\n", "utf-8"))

    def compute_control(self, error, t_d):
        # Compute control
        vel_control = self.p_gain * error

        # Clamp values between min and max speed
        vel_control = np.clip(
            vel_control,
            self.speed_min,
            self.speed_max,
        )

        self.pos += self.dir * vel_control * t_d

        # Clamp values between min and max position
        self.pos = np.clip(
            self.pos,
            self.pos_min,
            self.pos_max,
        )

        self.serial_write_angle(self.pos)
