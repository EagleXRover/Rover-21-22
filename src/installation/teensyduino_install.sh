#!/bin/bash

curDirTeensyduino=$(pwd)

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

cd

wget "https://downloads.arduino.cc/arduino-1.8.15-linux${systemType}.tar.xz"
wget "https://www.pjrc.com/teensy/td_154/TeensyduinoInstall.linux${systemType}"
wget "https://www.pjrc.com/teensy/00-teensy.rules"
sudo cp "00-teensy.rules" "/etc/udev/rules.d/"
tar -xf "arduino-1.8.15-linux${systemType}.tar.xz"
chmod 755 "TeensyduinoInstall.linux${systemType}"
"./TeensyduinoInstall.linux${systemType}" --dir=~/arduino/arduino-1.8.15
cd ~/arduino/arduino-1.8.15/hardware/teensy/avr/cores/teensy4
make



echo "Teesyduino installed successfully"

cd ${curDirTeensyduino}