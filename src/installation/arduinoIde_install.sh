#!/bin/bash

# Installs arduino IDE on the system.
echo "--------------------------------------------------"
echo "Installing arduino IDE"
sudo apt-get
sudo apt-get

curDir=$(pwd)
cd ~
mkdir arduino
cd arduino/
wget https://downloads.arduino.cc/arduino-1.8.15-linux64.tar.xz
tar -xvf ./arduino-1.8.15-linux64.tar.xz
cd arduino-1.8.15/
sudo ./install.sh
echo "Arduino IDE installed, setting up control for user $USER"
sudo usermod -a -G dialout $USER

cd $curDir