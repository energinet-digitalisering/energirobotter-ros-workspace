import numpy as np
import time

from servo_control.src import servo_coms
from servo_control.src import servo_control

# Initialise communicaiton and configuration setup.
servo = servo_control.ServoControl(
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_software_min=0,
    angle_max=180,
    angle_software_min=180,
    speed_max=200,
    dir=1,
    gain_P=1.0,
    gain_I=0.0,
    gain_D=0.0,
)

driver = servo_coms.ServoComs(
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_max=180,
    speed_max=200,
)

# Init driver
driver.init_arduino(port="/dev/ttyACM0")

# Set desired position in a continous loop

amp = 180
t_prev = time.time()

while True:
    t = time.time()

    angle_desired = np.sin(2 * (t)) * (amp / 2) + (amp / 2)
    angle, pwm = servo.reach_angle(t - t_prev, angle_desired)

    driver.write_angle(angle)

    t_prev = t
