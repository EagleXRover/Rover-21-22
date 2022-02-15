#!/bin/bash

# Saves previous directory as to be able to return to it
curDirArduinoIDE=$(pwd)
cd ~


# Installs arduino IDE on the system.
echo "--------------------------------------------------"
echo "Installing arduino IDE"

# Make sure your Ubuntu system packages are up-to-date:
sudo apt-get update
sudo apt-get upgrade -y

# Download Arduino IDE and extract
mkdir arduino
cd arduino/
wget https://downloads.arduino.cc/arduino-1.8.15-linux64.tar.xz

# Extract the tar.xz file:
tar -xvf ./arduino-1.8.15-linux64.tar.xz


# Install Arduino using the installer script
cd arduino-1.8.15/
sudo ./install.sh

# Adding user to dialout group
echo "Arduino IDE installed, setting up control for user ${USER}."
sudo usermod -a -G dialout ${USER}

# Arduino Ros Libraries
menu="1"
while [[ ${menu} == "1" ]]
do
    read -p "Install ros libraries? [Y/N] : " selection
    case ${selection,,} in
    "y" )
        cd ~/arduino/arduino-1.8.15/libraries
        rm -rf ros_lib
        rosrun rosserial_arduino make_libraries.py .
        echo "Arduino ROS lib installed successfully."
        menu="0";;
    "n" )
        menu="0";;
    esac
done

# Teesyuino
menu="1"
while [[ ${menu} == "1" ]]
do
    read -p "Install Teensyduino? [Y/N] : " selection
    case ${selection,,} in
    "y" )
        cd ~/catkin_ws/src/eaglex_rover/src/install
        ./teensyduino_install.sh
        menu="0";;
    "n" )
        menu="0";;
    esac
done

# Return to previos directory
cd ${curDirArduinoIDE}