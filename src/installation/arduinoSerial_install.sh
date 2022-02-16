#!/bin/bash

# Installs ROS serrial arduino on the system.
echo "--------------------------------------------------"
echo "Installing arduino ROS serial"
sudo apt-get install ros-$ROS_DISTRO-rosserial-arduino -y
sudo apt-get install ros-$ROS_DISTRO-rosserial -y

curDirArduinoSerial=$(pwd)
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git
cd ..
catkin_make
catkin_make install
source ~/catkin_ws/install/setup.bash
catkin_make
cd $curDirArduinoSerial