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
mkdir ~/arduino
cd ~/arduino/

menu="1"
while [[ $menu == "1" ]]
do
    echo 
    echo "-------------------------------------"
    echo "Which architecture does the device use?"
    echo "0 - x86 / 32 bit"
    echo "1 - x64 / 64 bit"
    echo "2 - ARM32 / Raspberry Pi"
    echo "3 - AARCH64 / Jetson TX2"
    read -p "Selection: " systemType
    case $systemType in
        0 ) 
            systemType="32"
            menu="0";;
        1 ) 
            systemType="64"
            menu="0";;
        2 ) 
            systemType="arm"
            menu="0";;
        3 ) 
            systemType="aarch64"
            menu="0";;
    esac
done

sudo rm -r "arduino-1.8.15-linux${systemType}.tar.xz"
wget "https://downloads.arduino.cc/arduino-1.8.15-linux${systemType}.tar.xz"

# Extract the tar.xz file:
tar -xvf ./arduino-1.8.15-linux64.tar.xz


# Install Arduino using the installer script
cd ~/arduino/arduino-1.8.15/
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
        sudo rm -rf ros_lib
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
        cd ~/arduino/
        sudo rm -r "TeensyduinoInstall.linux${systemType}"
        wget "https://www.pjrc.com/teensy/td_154/TeensyduinoInstall.linux${systemType}"
        sudo rm -r "00-teensy.rules"
        wget "https://www.pjrc.com/teensy/00-teensy.rules"
        
        sudo cp "00-teensy.rules" "/etc/udev/rules.d/"
        
        tar -xf "arduino-1.8.15-linux${systemType}.tar.xz"
        chmod 755 "TeensyduinoInstall.linux${systemType}";
        
        cd ~/arduino/arduino-1.8.15

        ~/arduino/TeensyduinoInstall.linux${systemType} --dir="."

        cd ~/arduino/arduino-1.8.15/hardware/teensy/avr/cores/teensy4
        
        make
        echo "Teesyduino installed successfully"

        menu="0";;
    "n" )
        menu="0";;
    esac
done


# Return to previos directory
cd ${curDirArduinoIDE}
