# Wattson Description

Robot URFD description of Wattson.

## Generate URDF

The URDF file and meshes are generated/exported from Blender 3.3, using the [Phobos addon](https://github.com/dfki-ric/phobos) (see [video guide](https://www.youtube.com/watch?v=JGPyNxzVlYA&t)). 

In the generated URDF, change all relative paths `../` with `package://wattson_description/`.

If simulating physics, add a `<dynamics damping="1.0" friction="1.0"/>` tag to all joints.


## Visualise URDF

With the `urdf_launch` ROS 2 package, the URDF file can be visualised and tested. Install dependency with:

```
rosdep install --from-paths src -y --ignore-src
```

And run with:
```
ros2 launch urdf_launch display.launch.py urdf_package:=wattson_description urdf_package_path:=urdf/wattson.urdf rviz_config:=src/energirobotter-ros-workspace/wattson_description/rviz/wattson_display.rviz
```