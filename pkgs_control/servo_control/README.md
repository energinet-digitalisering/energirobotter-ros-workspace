# Servo Control

Package implementing PID control and communication of/with servo motors, with Serial or I2C protocol through a servo driver, either a [Waveshare Driver](https://www.waveshare.com/servo-driver-with-esp32.htm), [PCA9685](https://www.adafruit.com/product/815), or any [Arduino](https://www.arduino.cc/) board.

See `examples/` for usage examples. Run examples from root through the terminal:
```
python3 src/energirobotter-vision/servo_control/examples/arduino_single_servo_example.py
```

## `ServoDriverWaveshare`

Driver for the [Waveshare Driver](https://www.waveshare.com/servo-driver-with-esp32.htm) component. Initialises and maintains communication. 

| Name     | Type     | Description | Default          |
| -------- | -------- | ----------- | ---------------- |
| port     | `string` | Port name.  | `"/dev/ttyACM0"` |
| baudrate | `int`    | Baudrate    | `115200`         |

## `ServoDriverPCA9685`

Driver for the [PCA9685](https://www.adafruit.com/product/815) component. Initialises and maintains communication. 

## `ServoDriverArduino`

Driver for an [Arduino](https://www.arduino.cc/) board. Initialises and maintains communication. 

| Name     | Type     | Description                       | Default          |
| -------- | -------- | --------------------------------- | ---------------- |
| port     | `string` | Port name.                        | `"/dev/ttyACM0"` |
| baudrate | `int`    | Baudrate                          | `115200`         |
| timeout  | `float`  | Timeout for serial communication. | `1.0`            |

## `ServoControlNode`

Node representing a single servo, using the ServoControl class to contain hardware configurations and functionality to move the servo. 

| Name               | Type     | Description                                                                                                                                                                                  | Default     |
| ------------------ | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- |
| servo_id           | `int`    | Servo id/channel on the PCA9685 board (only applicable for `driver_device:=pca9685`).                                                                                                        | `0`         |
| operation_mode     | `string` | Operation mode, similar to PPM and CSP EtherCAT operation modes. Requesting a position/angle, or sending error commands in a fixed time-loop. Supported protocols are `angle` and `control`. | `"angle"`   |
| driver_device      | `string` | Driver device name. Supported devices are `arduino` and `pca9685`.                                                                                                                           | `"pca9685"` |
| control_frequency  | `float`  | Control loop frequency                                                                                                                                                                       | `0.05`      |
| pwm_min            | `int`    | Servo minimum pwm (16-bit).                                                                                                                                                                  | `0`         |
| pwm_max            | `int`    | Servo maximum pwm (16-bit).                                                                                                                                                                  | `4095`      |
| angle_min          | `int`    | Servo minimum position in angles.                                                                                                                                                            | `0`         |
| angle_software_min | `int`    | Servo minimum dedesired position in angles, caued by physical configuration limits.                                                                                                          | `0`         |
| angle_max          | `int`    | Servo maximum position in angles.                                                                                                                                                            | `180`       |
| angle_software_max | `int`    | Servo maximum dedesired position in angles, caued by physical configuration limits.                                                                                                          | `180`       |
| speed_max          | `int`    | Servo maximum speed in angles/second.                                                                                                                                                        | `200`       |
| dir                | `int`    | Direction config for upside-down placement (-1 or 1).                                                                                                                                        | `1`         |
| gain_P             | `float`  | p-gain og PID controller.                                                                                                                                                                    | `1.0`       |
| gain_I             | `float`  | i-gain og PID controller.                                                                                                                                                                    | `0.0`       |
| gain_D             | `float`  | d-gain og PID controller.                                                                                                                                                                    | `0.0`       |

### PWM Calculation Example

The [Tower Pro SG90 servo](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf) runs at `50Hz` (PWM period of `20ms`). The minimum angle position is `1ms`, and the maximum angle is `2ms`, translating to a PWM percent between `5%` and `10%` for min and max position/angle. 

A 16-bit number has a maximum decimal value of 65535, so the min and max 16-bit PWM values for this servo is `3276.75` and `6553.5` (write as ints).


## `ServoControl`

Class representing a single servo, containing hardware configurations and functionality to move the servo. 

The `ServoControl` class is implemented in `servo_control.py`, and depends on `servo_coms.py` in the same directory, that handles the communication layer.

| Name               | Type     | Description                                                                         | Default          |
| ------------------ | -------- | ----------------------------------------------------------------------------------- | ---------------- |
| pwm_min            | `float`  | Servo minimum pwm (16-bit).                                                         | -                |
| pwm_max            | `float`  | Servo maximum pwm (16-bit).                                                         | -                |
| angle_min          | `float`  | Servo minimum position in angles.                                                   | -                |
| angle_software_min | `float`  | Servo minimum dedesired position in angles, caued by physical configuration limits. | -                |
| angle_max          | `float`  | Servo maximum position in angles.                                                   | -                |
| angle_software_max | `float`  | Servo maximum dedesired position in angles, caued by physical configuration limits. | -                |
| speed_max          | `float`  | Servo maximum speed in angles/second.                                               | -                |
| servo_id           | `int`    | Servo id/channel on the PCA9685 board (only applicable for I2C protocol).           | `0`              |
| dir                | `int`    | Direction config for upside-down placement (-1 or 1).                               | `1`              |
| gain_P             | `float`  | p-gain og PID controller.                                                           | `1.0`            |
| gain_I             | `float`  | i-gain og PID controller.                                                           | `0.0`            |
| gain_D             | `float`  | d-gain og PID controller.                                                           | `0.0`            |
| protocol           | `string` | Communications protocol. Supported protocols are `serial` and `i2c`.                | `"serial"`       |
| port               | `string` | Port name.                                                                          | `"/dev/ttyACM0"` |

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
