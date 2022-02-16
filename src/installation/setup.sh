#!/bin/bash

# Calls all other installation scripts for the system to run the package.
curDirSetup=$(pwd)

case ${ROS_DISTRO,,} in
    "noetic" )
        echo "ROS ${ROS_DISTRO,,} already installed";;
    "melodic" )
        echo "ROS ${ROS_DISTRO,,} already installed";;
    "" )
        bash <(curl -s https://raw.githubusercontent.com/EagleXRover/eaglex_rover/Package/src/installation/ROS_install.sh);;
esac

bash <(curl -s https://raw.githubusercontent.com/EagleXRover/eaglex_rover/Package/src/installation/package_install.sh)

cd ~/catkin_ws/src/eaglex_rover/src/installation

while true 
do
    echo $lineDivider
    read -p "Do you want to install Arduino INE [Y/N]: " ArduinoIDESelection
    ArduinoIDESelection=${ArduinoIDESelection,,}
    if [[ $ArduinoIDESelection != "" ]]
    then 
        if [[ $ArduinoIDESelection == "y" ]]
        then 
            ./arduinoIde_install.sh 
            break; 
        fi 
        if [[ $ArduinoIDESelection == "n" ]]
        then 
            break; 
        fi
    fi
done


cd ~/catkin_ws
catkin_make
catkin_make install
source ~/catkin_ws/install/setup.bash
catkin_make

echo "Basic setup installed... Hope you enjoy"

cd $curDirSetup