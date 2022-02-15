#!/bin/bash

curDirTeensyduino=$(pwd)

systemType="64"

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