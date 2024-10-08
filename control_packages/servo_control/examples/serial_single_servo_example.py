import numpy as np
import time

from servo_control.src import servo_control

# Initialise communicaiton and configuration setup.
servo = servo_control.ServoControl(
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_max=180,
    speed_max=200,
    servo_id=0,
    dir=1,
    protocol="serial",
)


# Set desired position in a continous loop

amp = 180
t_prev = time.time()

while True:
    t = time.time()

    angle_desired = np.sin(2 * (t)) * (amp / 2) + (amp / 2)

    servo.reach_angle(t - t_prev, angle_desired)
    t_prev = t
