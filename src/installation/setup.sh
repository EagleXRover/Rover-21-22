#!/bin/bash

# Calls all other installation scripts for the system to run the package.
curDir=$(pwd)

bash <(curl -s https://raw.githubusercontent.com/EagleXRover/eaglex_rover/main/src/installation/ROS_install.sh)
bash <(curl -s https://raw.githubusercontent.com/EagleXRover/eaglex_rover/main/src/installation/package_install.sh)


cd ~/catkin_ws/src/eaglex_rover/src/installation
sudo chmod 777 ROS_install.sh joy_install.sh arduinoSerial_install.sh arduinoIde_install.sh 

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

echo "Basic setup installed... Hope you enjoy"

cd $curDir