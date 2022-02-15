#!/bin/bash

# Keeps the directory so it can return to it afterwards.
curDir=$(pwd)

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash


cd ~/catkin_ws/src
git clone https://github.com/EagleXRover/eaglex_rover.git

cd ~/catkin_ws/src/eaglex_rover/src/installation

sudo chmod 777 ROS_install.sh joy_install.sh arduinoSerial_install.sh arduinoIde_install.sh 

./joy_install.sh
./arduinoSerial_install.sh


# Return to original directory.
cd $curDir