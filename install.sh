#!/bin/bash

# Install drone_slam ros package
cd $HOME && git clone https://github.com/iamarkaj/Drone-Mapping.git
ln -s $HOME/Drone-Mapping/drone_slam $HOME/catkin_ws/src
roscd && catkin_make

# Install gazebo_ros_velodyne_plugin
cd $HOME/Drone-Mapping/external/velodyne_gazebo_plugins
mkdir build && cd build
cmake ../
make

echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Drone-Mapping/drone_slam/models" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/Drone-Mapping/external/velodyne_gazebo_plugins/build/devel/lib" >> ~/.bashrc
