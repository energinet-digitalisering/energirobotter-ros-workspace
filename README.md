# Energirobotter ROS Workspace

Packages for Energinet's Humanoid Robots, part of the project "Energirobotter". 

- [Energirobotter ROS Workspace](#energirobotter-ros-workspace)
  - [Setup](#setup)
    - [Dialout Group](#dialout-group)
    - [Repository](#repository)
    - [Dependencies](#dependencies)
      - [ZED SDK](#zed-sdk)
        - [Ubuntu 22.04](#ubuntu-2204)
        - [Jetson Orin Nano (Jetpack 6.0)](#jetson-orin-nano-jetpack-60)
      - [ZED ROS 2 Wrapper](#zed-ros-2-wrapper)
    - [AI model](#ai-model)
    - [Build](#build)
  - [Usage](#usage)


## Setup

### Dialout Group

Add your user to the dialout/tty group on Linux:
```
sudo usermod -a -G dialout your_user_name
```

Reboot your system.

### Repository

Clone this repository into a `workspace/src/` folder:

```
git clone --recursive https://github.com/energinet-digitalisering/energirobotter-ros-workspace.git
```

Also clone other needed repos here:
```
git clone -b jazzy https://bitbucket.org/traclabs/trac_ik.git
```

Add an empty file called `COLCON_IGNORE` in the `src/trac_ik/trac_ik_kinematics_plugin/` folder, to not build the `MoveIt` plugin. 


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

#### ZED SDK

##### Ubuntu 22.04
Download and install [CUDA 12.6](https://developer.nvidia.com/cuda-downloads).

Download and install [ZED SDK v4.2](https://www.stereolabs.com/en-dk/developers/release) for CUDA 12. When prompted if the ZED SDK installer shall install CUDA, say no. 

##### Jetson Orin Nano (Jetpack 6.0)
Download and install [ZED SDK v4.2](https://www.stereolabs.com/en-dk/developers/release) for NVIDIA Jetson (ZED SDK for JetPack 6.0 GA (L4T 36.3)) 

#### ZED ROS 2 Wrapper

Follow the instructions on building the package in the [zed_ros2_wrapper](https://github.com/stereolabs/zed-ros2-wrapper?tab=readme-ov-file) repo. Name the `ros2_ws` folder something more appropriate, like `zed_wrapper_ws`. Also do the optional command of sourcing the workspace in `.bashrc`.

Replace the `zed2i.yaml` and `zedm.yaml` files in `~/zed_wrapper_ws/src/zed-ros2-wrapper/zed_wrapper/config/` with the versions provided in `energirobotter_bringup/config/zed_camera/` from this repository.

The `ZED_SDK` may have upgraded Numpy to 2.x, but ROS was built against Numpy 1.x, so it should be downgraded by running: `pip3 install "numpy<2" --force-reinstall`

### AI model
Download face detection model [yolov8n-face.pt](https://github.com/akanametov/yolov8-face/releases/download/v0.0.0/yolov8n-face.pt) from the [yolo-face repository](https://github.com/akanametov/yolo-face/tree/v0.0.0). Move the model into the `src/energirobotter-ros-workspace/pkgs_vision/face_detection/models/` directory.


### Build

Build `workspace` with:
```
colcon build --symlink-install
```

## Usage

Refer to the `README.md` in the `energirobotter_bringup` package for a description of the different launch files - aka. features.

