#!/bin/bash

# Keeps the directory so it can return to it afterwards.
curDir=$(pwd)


if [ !-d "~/catkin_ws/src"]
then
    mkdir -p ~/catkin_ws/src
fi
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

if [ !-d "~/catkin_ws/src/eaglex_rover"]
then
    cd ~/catkin_ws/src
    git clone https://github.com/EagleXRover/eaglex_rover.git
fi
cd ~/catkin_ws/src/eaglex_rover
git switch Package

cd ~/catkin_ws/src/eaglex_rover/src/installation

sudo chmod 777 ROS_install.sh joy_install.sh arduinoSerial_install.sh arduinoIde_install.sh 

./joy_install.sh
./arduinoSerial_install.sh


# Return to original directory.
cd $curDir