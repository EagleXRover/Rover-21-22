#!/bin/bash

# Installs ROS serrial arduino on the system.
sudo apt-get install ros-$ROS_DISTRO-rosserial-arduino
sudo apt-get install ros-$ROS_DISTRO-rosserial

curDir=$(pwd)
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git
cd ..
catkin_make
catkin_make install
cd $pwd