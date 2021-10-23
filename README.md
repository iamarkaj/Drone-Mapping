# Drone 3D-Mapping & Navigation *(WIP)*

![](https://img.shields.io/badge/Ubuntu-20.04-red)
![](https://img.shields.io/badge/ROS1-Noetic-blue)
![](https://img.shields.io/badge/Gazebo-11-green)

## Drone Controller Teaser

<a href="https://youtu.be/L2CJCvOHqHQ">
<img src="drone_controller/resources/controller.gif" alt="stream" width="600"/>
</a>

## Usage

```bash
cd $HOME && git clone https://github.com/iamarkaj/Drone-Mapping.git
ln -s $HOME/Drone-Mapping/drone_slam $HOME/catkin_ws/src
roscd && catkin_make
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Drone-Mapping/drone_slam/models" >> ~/.bashrc
```

```bash
roslaunch drone_slam drone_slam.launch
```

```bash
python $HOME/Drone-Mapping/drone_controller/widget/widget.py
```

## Important links

[Fix axis for pointcloud](https://github.com/sachazyto/gazebo_ros_pkgs/commit/1a038579dd256505a1786a4e8566dab25ec2f48f#diff-a7fc4c55eaaae0cc1ecc84e1f7e2fa72599aa654b28a0e6807e00524ff7c6c24)