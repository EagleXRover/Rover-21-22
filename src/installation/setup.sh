#!/bin/bash

# Calls all other installation scripts for the system to run the package.
curDir=$(pwd)
cd ~/catkin_make/src/eaglex_rover/src/installation
sudo chmod 777 arduino_install.sh
sudo chmod 777 joy_install.sh

./arduino_install.sh
./joy_install.sh

echo "Dont forget to run catkin_make on your <ws> folder."

cd $curDir