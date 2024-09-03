# Servo Control

Package implementing PID control and communication of/with servo motors, with Serial or I2C protocol.

See `servo_control_example.ipynb` for usage example.


## `ServoControl`

Class representing a single servo, containing hardware configurations and functionality to move the servo. 

The `ServoControl` class is implemented in `servo_control.py`, and depends on `servo_coms.py` in the same directory, that handles the communication layer.

| Name      | Type     | Description                                                               | Default          |
| --------- | -------- | ------------------------------------------------------------------------- | ---------------- |
| pwm_min   | `float`  | Servo minimum pwm.                                                        | -                |
| pwm_max   | `float`  | Servo maximum pwm.                                                        | -                |
| angle_min | `float`  | Servo minimum position in angles.                                         | -                |
| angle_max | `float`  | Servo maximum position in angles.                                         | -                |
| speed_max | `float`  | Servo maximum speed in angles/second.                                     | -                |
| servo_id  | `int`    | Servo id/channel on the PCA9685 board (only applicable for I2C protocol). | `0`              |
| dir       | `int`    | Direction config for upside-down placement (-1 or 1).                     | `1`              |
| gain_P    | `float`  | p-gain og PID controller.                                                 | `1.0`            |
| gain_I    | `float`  | i-gain og PID controller.                                                 | `0.0`            |
| gain_D    | `float`  | d-gain og PID controller.                                                 | `0.0`            |
| protocol  | `string` | Communications protocol. Supported protocols are `serial` and `i2c`.      | `"serial"`       |
| port      | `string` | Port name.                                                                | `"/dev/ttyACM0"` |

> To use I2C protocol, the code must be run on a board with I2C pins.



## Member Funcitons

Three functions are relevant for users, all of which sould be called prediodically with known delta time, for smooth movement.

### `compute_control`

Computing and applying PID control. 
The error does not need to be in angles, and is used by face_following where the error represents pixels. The gains are tuned acordingly.  


#### Parameters
| Name          | Type    | Description                                                            | Default |
| ------------- | ------- | ---------------------------------------------------------------------- | ------- |
| t_d           | `float` | Delta time from last loop.                                             | -       |
| error         | `float` | Error inout to the PID controller, it is `reference - current`.        | -       |
| speed_desired | `float` | Desired speed in angles pr. second, value of `-1` indicates top speed. | `-1`    |


### `reach_angle`

Reach desirered angle position, speed will be top speed.

#### Parameters
| Name  | Type    | Description                       | Default |
| ----- | ------- | --------------------------------- | ------- |
| t_d   | `float` | Delta time from last loop.        | -       |
| angle | `float` | Desired angle position (degrees). | -       |


### `reset_position`

Reset to default position (angle of 90 degrees).

#### Parameters

| Name | Type    | Description                | Default |
| ---- | ------- | -------------------------- | ------- |
| t_d  | `float` | Delta time from last loop. | -       |
