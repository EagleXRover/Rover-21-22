#!/bin/bash

# Installs ROS joy on the system.
sudo apt-get install ros-$ROS_DISTRO-joy

# Configures joystick for correct usage.
echo " "
echo "---------------------------------------------"
echo "---------------------------------------------"
echo " "
echo "Please connect your joystick and then press enter on keyboard."
read joystick

ls /dev/input/
echo "select your joystick (Generally is jsX where X is a number): "
read joystick
sudo chmod a+rw /dev/input/$joystick

echo " "
echo "---------------------------------------------"
echo "---------------------------------------------"
echo " "
echo "ROS joy installed, and joystick setup completed."
echo "If you want to test your joystick, you can run the next command: "
echo "'sudo jstest /dev/input/${joystick}'"

echo ""
echo "or procede with the ROS Joy setup with the next commands: "
echo "'roscore'"
output='"' 
echo "'rosparam set joy_node/dev ${output}/dev/input/${joystick}${output}'"
echo "'rosrun joy joy_node'"
echo "and in a new terminal run this last command: "
echo "'rostopic echo joy'"