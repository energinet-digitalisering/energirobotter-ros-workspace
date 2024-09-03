# Energirobotter Vision

Vision capabilities and camera handling for the Humanoid Robot project "Energirobotter". 

## Setup

Clone this repository into a `workspace/src/` folder, along with [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros2-master).

Python modules not included in [rosdistro](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml) can be installed from root of workspace with 
```
pip install -r src/energirobotter-vision/requirements.txt
```

## Usage

Use the `vision_bringup` package's `vision.launch.py` to start the camera and face detection:

```
ros2 launch vision_bringup vision.launch.py use_compressed:=true
```


