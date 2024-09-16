import numpy as np
import time

from servo_control.src import servo_control

servo_0 = servo_control.ServoControl(
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_max=180,
    speed_max=200,
    servo_id=0,
    dir=1,
    protocol="i2c",
)

servo_1 = servo_control.ServoControl(
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_max=180,
    speed_max=200,
    servo_id=1,
    dir=1,
    protocol="i2c",
)

servo_2 = servo_control.ServoControl(
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_max=180,
    speed_max=200,
    servo_id=2,
    dir=1,
    protocol="i2c",
)

servo_3 = servo_control.ServoControl(
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_max=180,
    speed_max=200,
    servo_id=3,
    dir=1,
    protocol="i2c",
)


# Set desired position in a continous loop

amp = 90
offset = 90
t_prev = time.time()

while True:
    t = time.time()
    t_d = t - t_prev

    angle_0 = np.sin(2 * (t + 0 * t / 4)) * amp + offset
    angle_1 = np.sin(2 * (t + 1 * t / 4)) * amp + offset
    angle_2 = np.sin(2 * (t + 2 * t / 4)) * amp + offset
    angle_3 = np.sin(2 * (t + 3 * t / 4)) * amp + offset

    servo_0.reach_angle(t_d, angle_0)
    servo_1.reach_angle(t_d, angle_1)
    servo_2.reach_angle(t_d, angle_2)
    servo_3.reach_angle(t_d, angle_3)

    t_prev = t
