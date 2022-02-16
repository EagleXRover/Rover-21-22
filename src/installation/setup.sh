#!/bin/bash

# Calls all other installation scripts for the system to run the package.
curDirSetup=$(pwd)

# Solves the problem of changing time when changing OS.
timedatectl set-local-rtc 1

# Downloads the package as to be able to work on it.
if [ !-d "~/catkin_ws/src"]
then
    mkdir -p ~/catkin_ws/src
fi

if [ !-d "~/catkin_ws/src/eaglex_rover"]
then
    cd ~/catkin_ws/src
    git clone https://github.com/EagleXRover/eaglex_rover.git
fi

cd ~/catkin_ws/src/eaglex_rover/
git switch Package
cd ~/catkin_ws/src/eaglex_rover/src/installation
sudo chmod 777 setup.sh ROS_install.sh joy_install.sh package_install.sh arduinoSerial_install.sh arduinoIde_install.sh teensyduino_install.sh


# Searches for ROS distro on the system, and if there is non, installs the respective distro.
case ${ROS_DISTRO,,} in
    "noetic" )
        echo "ROS ${ROS_DISTRO,,} already installed";;
    "melodic" )
        echo "ROS ${ROS_DISTRO,,} already installed";;
    "" )
        bash <(curl -s https://raw.githubusercontent.com/EagleXRover/eaglex_rover/Package/src/installation/ROS_install.sh);;
esac

# Searches for the package on the system, if it's not on the system, it procedes to install it.
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