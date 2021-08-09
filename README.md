# Drone 3D-Mapping & Navigation *(WIP)*

![](https://img.shields.io/badge/Ubuntu-20.04-red)
![](https://img.shields.io/badge/ROS1-Noetic-blue)
![](https://img.shields.io/badge/Gazebo-11-green)

## Teaser

<a href="https://youtu.be/L2CJCvOHqHQ">
<img src="drone_controller/resources/controller.gif" alt="stream" width="600"/>
</a>

## Usage

```bash
git clone https://github.com/iamarkaj/Drone-Mapping.git
ln -s ~/Drone-Mapping/assets ~/catkin_ws/src
roscd && catkin build
```

```bash
roslaunch ~/Drone-Mapping/drone_mapping.launch
```

```bash
python ~/Drone-Mapping/drone_controller/widget/widget.py
```