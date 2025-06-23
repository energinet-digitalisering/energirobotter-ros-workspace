# Energirobotter ROS Workspace

Packages for Energinets Humanoid Robots, part of the project "Energirobotter". 

## Setup

### Dialout Group

Add your user to the dialout/tty group on Linux:
```
sudo usermod -a -G dialout your_user_name
```

Reboot your system.

### Repository

Clone this repository into a `workspace/src/` folder, along with [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper):

```
cd src
git clone git@github.com:energinet-digitalisering/energirobotter-ros-workspace.git

git clone --branch humble-v4.1.4 --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
```

### ZED SDK

#### Ubuntu 22.04
Download and install [CUDA 12.6](https://developer.nvidia.com/cuda-downloads).

Download and install [ZED SDK v4.2](https://www.stereolabs.com/en-dk/developers/release) for CUDA 12. When prompted if the ZED SDK installer shall install CUDA, say no. 

#### Jetson Orin Nano (Jetpack 6.0)
Download and install [ZED SDK v4.2](https://www.stereolabs.com/en-dk/developers/release) for NVIDIA Jetson (ZED SDK for JetPack 6.0 GA (L4T 36.3)) 



### Dependencies

In `worspace` root, source ROS and install ROS dependencies with rosdep:
```
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Python modules not included in [rosdistro](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml) can be installed from root of workspace with:
```
pip install -r src/energirobotter-ros-workspace/requirements.txt
```

### AI model
Download face detection model [yolov8n-face.pt](https://github.com/akanametov/yolov8-face/releases/download/v0.0.0/yolov8n-face.pt) from the [yolo-face repository](https://github.com/akanametov/yolo-face/tree/v0.0.0). Move the model into the `src/energirobotter-ros-workspace/pkgs_vision/face_detection/models/` directory.


### Build

Build `workspace` with:
```
colcon build --symlink-install
```

## Usage

Use the `energirobotter_bringup` package's `vision.launch.py` to start the camera and face detection:

```
source install/setup.bash
ros2 launch energirobotter_bringup vision.launch.py use_compressed:=true
```

If no camera is available, run example with:
```
ros2 launch energirobotter_bringup vision.launch.py use_mock_camera:=true
```

> Note: The first time `face_detection` is run `ultralytics` will install som extra packages, and require a restart of the node.

