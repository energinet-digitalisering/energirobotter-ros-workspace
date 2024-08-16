# Energirobotter Vision

Vision capabilities and camera handling for the Humanoid Robot project "Energirobotter". 

Clone this repository into a `workspace/src/` folder, along with [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros2-master).

Use the `vision_bringup` package's `vision.launch.py` to start the camera and face detection:

```
ros2 launch vision_bringup vision.launch.py use_compressed:=true
```


