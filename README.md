# Drone 3D-Mapping & Navigation *(WIP)*

![](https://img.shields.io/badge/Ubuntu-20.04-red)
![](https://img.shields.io/badge/ROS1-Noetic-blue)
![](https://img.shields.io/badge/Gazebo-11-green)

## Usage

```bash
git clone https://github.com/iamarkaj/Drone-Mapping.git
ln -s ~/Drone-Mapping/assets ~/catkin_ws/src
roscd && catkin build
```

```bash
cd ~/Drone-Mapping
roslaunch drone_mapping.launch
```

```bash
cd ~/Drone-Mapping/drone_controller/widget
python widget.py
```