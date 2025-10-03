# Wattson Description

Robot URFD description of Wattson.

## Generate URDF

The URDF file and meshes are generated/exported from Blender 3.3, using the [Phobos addon](https://github.com/dfki-ric/phobos) (see [video guide](https://www.youtube.com/watch?v=JGPyNxzVlYA&t), especially [Part 2](https://www.youtube.com/watch?v=CrMvtlZl3LI&ab_channel=FatAl) is relevant). 

Here's a quick rundown:

1. **Prepare the robot model in Blender**  
   Start by cleaning up the assembly: remove unnecessary parts such as screws and bolts — only visually significant components of the robot links are needed. Optionally, you can join parts that belong to the same link to simplify the model. Position the robot so that:  
   - The torso origin is placed at the world origin (`0, 0, 0`)  
   - The robot stands upright  
   - The robot faces the positive Y-axis  

2. **Set visual properties**  
   Select all parts of the robot and set their **Phobostype** to `Visual`. Define geometry as `mesh`. 

3. **Create collision geometry**  
   Create collision objects as in the video with primitive types. 

4. **Define links and align them**  
   Create a link for each part. Align each link so its Z-axis corresponds to the intended joint rotation axis. When rotating around the Z-axis:  
   - **Roll** is positive when moving away from the robot’s body  
   - **Pitch** is positive when moving in front of the robot  
   - **Yaw** is positive when turning away from the robot (as if the front face is turning outward)  
   
   Use the **right-hand rule** to ensure the positive rotation around each link’s Z-axis matches these conventions. 

   Link names should have a `link_` prefix to match convention, and the end-effectors (last link in a chain) should be named `link_left_hand`, `link_right_hand`, and `link_head_roll`. 
   > If these need to be changed, changes should also be made in the IK package. 

5. **Name the robot model**  
   Assign a name to the robot — this will set the root-link.

6. **Define joints**  
  Select all links and use the *Define Joints* function. Set the joint type to `Revolute`.  
   - Create appropriate limits for all joints.

1. **Name joints**  
   Name each joint to match the corresponding servo joint names from the servo configuration file.

2. **Export the model**  
   Set the export path to `//`. In the export settings, enable:  
   - `Export Textures`  
   - `URDF`  
   - `STL`  
   
   In the URDF export tab:  
   - Set mesh type to `STL`  
   - Use **relative file paths**  
   Finally, click **Export Model**.

3. **Finalise URDF**  
   - This package has a `fetch_phobos_urdf.py` script in the `utils/` folder, to fetch the URDF generated from Phobos and change the needed lines. 
   Change the paths in the `source_file` and `destination_file` variables, then run the script.
   -  Manually copy the mesh files from the export folder to the `meshes/stl/` folder in the description package.

   > Should the hand orientations look weird when run with teleoperation, it is because of a transform from the VR controllers to the robot hands. Tune `self.hand2gripper_left` and `self.hand2gripper_right` in the `TrackingTransformer` class, found in `pkgs_teleoperation/teleoperation/teleoperation/src/tracking_transformer.py`.




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

| Parameter name       | Description                                                                 | Datatype |
| -------------------- | --------------------------------------------------------------------------- | -------- |
| `servo_id`           | Unique identifier for the servo.                                            | `int`    |
| `dir`                | Direction configuration for upside-down placement (1 or -1). Defaults to 1. | `int`    |
| `gear_ratio`         | Gear ratio for linked mechanisms. Defaults to 1.                            | `int`    |
| `pwm_min`            | Minimum PWM value.                                                          | `int`    |
| `pwm_max`            | Maximum PWM value.                                                          | `int`    |
| `angle_min`          | Minimum physical angle of the servo.                                        | `float`  |
| `angle_max`          | Maximum physical angle of the servo.                                        | `float`  |
| `angle_software_min` | Minimum software-limited angle.                                             | `float`  |
| `angle_software_max` | Maximum software-limited angle.                                             | `float`  |
| `angle_speed_max`    | Maximum angular speed (degrees/second).                                     | `float`  |
| `default_position`   | Default angle position. Defaults to 180.                                    | `float`  |
| `feedback_enabled`   | Whether feedback control is enabled. Defaults to False.                     | `bool`   |
| `gain_P`             | Proportional gain for PID control. Defaults to 1.0.                         | `float`  |
| `gain_I`             | Integral gain for PID control. Defaults to 0.0.                             | `float`  |
| `gain_D`             | Derivative gain for PID control. Defaults to 0.0.                           | `float`  |




### Waveshare ST3215 Servo Notes

These servos are used in the arms.
They are controlled using the `sms_sts` class in the `SCServo_Python` module.

Waveshare ST3215 servos PWM is represented by a 12-bit number, so in base 10 it is `0 - 4095`. Tests showed that the actual range accepted by the driver was `0 - 4094`.

When sending a direct command to a Waveshare servo in code, one can set `SCS_MOVING_SPEED` and `SCS_MOVING_ACC`. In the contorl code in this repo, both are set to a constant value (typically max), as the speed and acceleration are controlled through sending specific positions. But for reference and tests, the maximum value for `SCS_MOVING_SPEED` seems to be `4000` (unknown unit) and for `SCS_MOVING_ACC` it is `255` (will crash the program if it goes beyond). Setting `SCS_MOVING_SPEED` to `0` will set it to the maximum value.

Waveshare servos can rotate from `0°` to `360°` and have a configurable **middle-point**, corresponding to `180°`. This middle-point acts as a reference around which the servo moves. Choosing the correct middle-point is especially important when the servo is **geared**, and its range of motion is reduced. The middle-point of a servo is configured through the web-app hosted by the servo driver.

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

#### Tuning

Use the `command_test_node` in the `servo_control` package to test different angles of a servo using the control pipeline. Launch `servos.launch.py` on the computer connected to the servo driver board: 

```
ros2 launch energirobotter_bringup servos.launch.py
```

Run the test node on the same subnet to send a one-time command:
```
ros2 run servo_control command_test_node --ros-args -p topic_name:=/joint_states -p joint_name:=joint_left_wrist_pitch -p angle:=20
```

> When a servo’s direction is flipped (i.e., `dir = -1`), take extra care when setting the `angle_software_min` and `angle_software_max` attributes. The effective angle range may be reversed compared to non-flipped servos — what is `angle_software_min` on a standard servo might correspond to `angle_software_max` on a flipped one, and vice versa.  `angle_software_min` is always less than `angle_software_max`.

### Waveshare SC09 Servo Notes

These servos are used in the hands.
They are controlled using the `scscl` class in the `SCServo_Python` module.

These servos have a middle-point just like the ST3115 servos, but it is not configurable. Make sure they are installed whith this in mind. 

The hand servos are scaled to a `0 - 90` degree range. Meaning sending an angle command of `0` will result in `angle_software_min`, and `90` will be `angle_software_max`. For tuning it can be nice to turn this off, do this by launching `servos.launch.py` with the `finger_mapping_enabled` parameter set to false:

```
ros2 launch energirobotter_bringup servos.launch.py finger_mapping_enabled:=false
```

The hand servos operate on a seperate joint state topic that the rest of the robot (as they don't use IK), remember to run the `command_test_node` with the correct topic name:
```
ros2 run servo_control command_test_node --ros-args -p topic_name:=/joint_states_hands -p joint_name:=hand_left_index -p angle:=20
```

