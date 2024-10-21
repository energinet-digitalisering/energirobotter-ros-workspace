import numpy as np
import time

from servo_control.src import servo_control

soft_min_0 = 70
soft_max_0 = 124
dir_0 = 1

servo_0 = servo_control.ServoControl(
    servo_id=0,
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_software_min=soft_min_0,
    angle_max=180,
    angle_software_max=soft_max_0,
    speed_max=200,
    dir=dir_0,
    protocol="i2c",
)

soft_min_1 = 68
soft_max_1 = 116
dir_1 = -1

servo_1 = servo_control.ServoControl(
    servo_id=1,
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_software_min=soft_min_1,
    angle_max=180,
    angle_software_max=soft_max_1,
    speed_max=200,
    dir=dir_1,
    protocol="i2c",
)

soft_min_2 = 68
soft_max_2 = 116
dir_2 = -1

servo_2 = servo_control.ServoControl(
    servo_id=2,
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_software_min=soft_min_2,
    angle_max=180,
    angle_software_max=soft_max_2,
    speed_max=200,
    dir=dir_2,
    protocol="i2c",
)

soft_min_3 = 80
soft_max_3 = 124
dir_3 = 1

servo_3 = servo_control.ServoControl(
    servo_id=3,
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_software_min=soft_min_3,
    angle_max=180,
    angle_software_max=soft_max_3,
    speed_max=200,
    dir=dir_3,
    protocol="i2c",
)

soft_min_4 = 80
soft_max_4 = 108
dir_4 = -1

servo_4 = servo_control.ServoControl(
    servo_id=4,
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_software_min=soft_min_4,
    angle_max=180,
    angle_software_max=soft_max_4,
    speed_max=200,
    dir=dir_4,
    protocol="i2c",
)

# Set desired position in a continous loop

amp_0 = (soft_max_0 - soft_min_0) / 2
offset_0 = soft_min_0 + amp_0

amp_1 = (soft_max_1 - soft_min_1) / 2
offset_1 = soft_min_1 + amp_1

amp_2 = (soft_max_2 - soft_min_2) / 2
offset_2 = soft_min_2 + amp_2

amp_3 = (soft_max_3 - soft_min_3) / 2
offset_3 = soft_min_3 + amp_3

amp_4 = (soft_max_4 - soft_min_4) / 2
offset_4 = soft_min_4 + amp_4


speed_mult = 2
t_prev = time.time()


while True:
    t = time.time()
    t_d = t - t_prev

    angle_0 = np.sin(speed_mult * (t + 0 * t / 5)) * amp_0 + offset_0
    angle_1 = np.sin(speed_mult * (t + 1 * t / 5)) * amp_1 + offset_1
    angle_2 = np.sin(speed_mult * (t + 2 * t / 5)) * amp_2 + offset_2
    angle_3 = np.sin(speed_mult * (t + 3 * t / 5)) * amp_3 + offset_3
    angle_4 = np.sin(speed_mult * (t + 4 * t / 5)) * amp_4 + offset_4

    servo_0.reach_angle(t_d, angle_0)
    servo_1.reach_angle(t_d, angle_1)
    servo_2.reach_angle(t_d, angle_2)
    servo_3.reach_angle(t_d, angle_3)
    servo_4.reach_angle(t_d, angle_4)

    t_prev = t
