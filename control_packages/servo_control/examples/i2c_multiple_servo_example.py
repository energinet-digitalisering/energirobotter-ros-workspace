import numpy as np
import time

from servo_control.src import servo_control


soft_min_0 = 53
soft_max_0 = 90
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


soft_min_1 = 90
soft_max_1 = 125
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


soft_min_2 = 85
soft_max_2 = 125
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
soft_max_3 = 120
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


soft_min_4 = 100
soft_max_4 = 140
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


soft_min_5 = 100
soft_max_5 = 140
dir_5 = 1

servo_5 = servo_control.ServoControl(
    servo_id=5,
    pwm_min=2400,
    pwm_max=9600,
    angle_min=0,
    angle_software_min=soft_min_5,
    angle_max=180,
    angle_software_max=soft_max_5,
    speed_max=200,
    dir=dir_5,
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

amp_5 = (soft_max_5 - soft_min_5) / 2
offset_5 = soft_min_5 + amp_5


speed_mult = 3
t_prev = time.time()


wave_factor = 2

while True:
    t = time.time()
    t_d = t - t_prev

    angle_0 = np.sin(speed_mult * t) * amp_0 + offset_0
    angle_1 = np.sin(speed_mult * (t + wave_factor * 0.1)) * amp_1 + offset_1
    angle_2 = np.sin(speed_mult * (t + wave_factor * 0.3)) * amp_2 + offset_2
    angle_3 = np.sin(speed_mult * (t + wave_factor * 0.2)) * amp_3 + offset_3
    angle_4 = np.sin(speed_mult * (t + wave_factor * 0.4)) * amp_4 + offset_4
    angle_5 = np.sin(speed_mult * (t + wave_factor * 0.5)) * amp_5 + offset_5

    servo_0.reach_angle(t_d, angle_0)
    servo_1.reach_angle(t_d, angle_1)
    servo_2.reach_angle(t_d, angle_2)
    servo_3.reach_angle(t_d, angle_3)
    servo_4.reach_angle(t_d, angle_4)
    servo_5.reach_angle(t_d, angle_5)

    t_prev = t
