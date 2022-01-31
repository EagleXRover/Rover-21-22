#!/bin/bash

# Installs ROS environment to the system.

curDir=$(pwd)

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update -y

lineDivider="--------------------------------------------------"

echo $lineDivider
while true 
do
    read -p "Which ROS version do you want to install? (for example 'noetic'): " ROSVERSION
    if [[ $ROSVERSION != "" ]]
    then 
        ROSVERSION=${ROSVERSION,,}
        break; 
    fi
done

while true 
do
    echo " "
    echo $lineDivider
    echo "How much of ROS you would like to install?"
    echo "0 - Bare Bones"
    echo "1 - Base"
    echo "2 - Full"
    read -p "Selection: " ROSIZE
    if [[ $ROSIZE != "" ]]
    then 
        if [[ $ROSIZE == "0" ]]
        then 
            ROSIZE="ros-base"
            break; 
        fi 
        if [[ $ROSIZE == "1" ]]
        then 
            ROSIZE="desktop"
            break; 
        fi 
        if [[ $ROSIZE == "2" ]]
        then 
            ROSIZE="desktop-full"
            break; 
        fi 
    fi
done

sudo apt install ros-$ROSVERSION-$ROSIZE -y
source /opt/ros/$ROSVERSION/setup.bash
echo "source /opt/ros/$ROSVERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc

UbuntuRelease=$(lsb_release -r -s)
if [[ ($UbuntuRelease < 20)]]
then
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
else
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
fi

rosdep init
rosdep update


cd $curDir