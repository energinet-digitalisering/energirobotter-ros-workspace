import numpy as np
import time

from servo_control import servo_control

# Initialise communicaiton and configuration setup.
servo = servo_control.ServoControl(
    pwm_min=150,
    pwm_max=600,
    angle_min=0,
    angle_max=180,
    speed_max=200,
    servo_id=0,
    dir=1,
    protocol="serial",
)


# Set desired position in a continous loop
t_prev = time.time()
while True:
    t = time.time()

    angle_desired = np.sin(2 * (t)) * 90 + 90

    servo.reach_angle(t - t_prev, angle_desired)
    t_prev = t
