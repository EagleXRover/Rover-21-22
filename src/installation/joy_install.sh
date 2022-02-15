#!/bin/bash

joySetup(){}

# Installs ROS joy on the system.
curDir=$(pwd)
sudo apt-get install ros-$ROS_DISTRO-joy -y

# Configures joystick for correct usage.
echo
echo "--------------------------------------------------------------------------"
read -p "Please connect your joystick and then press enter on the keboard." joystick

ls /dev/input

joystick=""
while [[ $joystick == "" ]]
do
    read -p "Plase select your joystick (joystick devices are referred to by jsX): " joystick
done

echo "Selected Joy = $joystick"
sudo chmod a+rw /dev/input/$joystick


echo "----------------------------------------------------------------------"
echo "----------------------------------------------------------------------"
echo "ROS Joy installed, and joystick setup completed."
echo "If you want to test your joystick, you can run the folowing command:"
echo "'sudo jtest /dev/input/$joystick'"
echo "----------------------------------------------------------------------"
echo "----------------------------------------------------------------------"

rosparam set joy_node/dev "/dev/input/$joystick"

echo -e "\nROS Joy installed and setup succesfully."

cd $curDir