# Wattson Description

Robot URFD description of Wattson.

## Generate URDF

The URDF file and meshes are generated/exported from Blender 3.3, using the [Phobos addon](https://github.com/dfki-ric/phobos) (see [video guide](https://www.youtube.com/watch?v=JGPyNxzVlYA&t)). 

In the generated URDF, change all relative paths `../` with `package://wattson_description/`.

If simulating physics, add a `<dynamics damping="1.0" friction="1.0"/>` tag to all joints.

As a shortcut, run the `fetch_phobos_urdf.py` script in the `utils/` folder, to fetch the URDF generated from Phobos and change the needed lines. Change the paths in the `source_file` and `destination_file` variables. 


## Visualise URDF

With the `urdf_launch` ROS 2 package, the URDF file can be visualised and tested. Install dependency with:

```
rosdep install --from-paths src -y --ignore-src
```

And run with:
```
ros2 launch urdf_launch display.launch.py urdf_package:=wattson_description urdf_package_path:=urdf/wattson.urdf rviz_config:=src/energirobotter-ros-workspace/wattson_description/rviz/wattson_display.rviz
```

## Servo Configuration


The `servo_manager_node` loads `.json` files that describes all parameters for each servo in a chain. Parameters to set are seen in the table below.

| Parameter name       | Description                                                                 | Datatype      |
| -------------------- | --------------------------------------------------------------------------- | ------------- |
| `servo_id`           | Unique identifier for the servo.                                            | `unsiged int` |
| `dir`                | Direction configuration for upside-down placement (1 or -1). Defaults to 1. | `int`         |
| `gear_ratio`         | Gear ratio for linked mechanisms. Defaults to 1.                            | `int`         |
| `pwm_min`            | Minimum PWM value.                                                          | `int`         |
| `pwm_max`            | Maximum PWM value.                                                          | `int`         |
| `angle_min`          | Minimum physical angle of the servo.                                        | `float`       |
| `angle_max`          | Maximum physical angle of the servo.                                        | `float`       |
| `angle_software_min` | Minimum software-limited angle.                                             | `float`       |
| `angle_software_max` | Maximum software-limited angle.                                             | `float`       |
| `angle_speed_max`    | Maximum angular speed (degrees/second).                                     | `float`       |
| `default_position`   | Default angle position. Defaults to 180.                                    | `float`       |
| `feedback_enabled`   | Whether feedback control is enabled. Defaults to False.                     | `bool`        |
| `gain_P`             | Proportional gain for PID control. Defaults to 1.0.                         | `float`       |
| `gain_I`             | Integral gain for PID control. Defaults to 0.0.                             | `float`       |
| `gain_D`             | Derivative gain for PID control. Defaults to 0.0.                           | `float`       |




### Notes

Waveshare servos can rotate from `0°` to `360°` and have a configurable **middle-point**, corresponding to `180°`. This middle-point acts as a reference around which the servo moves. Choosing the correct middle-point is especially important when the servo is **geared**, and its range of motion is reduced.

---

#### 1. Understanding the Middle-Point

- The **middle-point** (`180°`) is the reference angle used to center the servo’s motion.
- If the servo is geared (e.g., a `4:1` ratio), its effective movement range is reduced. For example, a `4:1` ratio results in a `90°` movement range.
- To make the most of this limited range, set the middle-point halfway between the **relaxed** and **fully flexed** positions of the joint.

**Example:**
> An elbow joint has a gear ratio of `4:1` → `90°` total movement range.  
> To reach both relaxed and flexed positions, set the middle-point at the angle halfway between them.

---

#### 2. Setting `angle_software_min` and `angle_software_max`

These parameters define the servo's **allowed motion range** in software.  
They must always stay within the physical range (`angle_min` to `angle_max`, usually `0–360°`).

**Formula:**
```
angle_software_min = middle_point - (motion_range / 2)
angle_software_max = middle_point + (motion_range / 2)
```

**Example:**
- Middle-point: `180°`
- Gear ratio: `4:1` → motion range: `90°`
- Calculation:
  - `angle_software_min = 180 - 90 / 2 = 135`
  - `angle_software_max = 180 + 90 / 2 = 225`

---

#### 3. Choosing the Default Position

The **default position** is the servo's starting/resting position after power-on or reset.

- It does **not** have to be the middle-point.
- Choose a position that reflects the **natural resting pose** of the robot.

**Example:**
> If the elbow naturally rests in the relaxed position, and that corresponds to `135°`,  
> then set the default position to `135°`, not `180°`.

---
