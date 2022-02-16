#!/bin/bash

# Keeps the directory so it can return to it afterwards.
curDirROS=$(pwd)

# Defines which version of ROS to use.
case $(lsb_release -r -s) in
    "20.04" )
        rosVersion="noetic";;
    "18.04" )
        rosVersion="melodic";;
esac


# Setup your computer to accept software from packages.ros.org. 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup your keys.
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installation.
sudo apt update

menu="1"
while [[ $menu == "1" ]]
do 
    echo
    echo "-------------------------------------------"
    echo "How much of ROS would you like to install?"
    echo "0 - Bare Bones"
    echo "1 - Base"
    echo "2 - Full"
    read -p "Selection: " rosType
    case $rosType in
        0 )
            rosType="ros-base"
            menu="0";;
        1 ) 
            rosType="desktop"
            menu="0";;
        2 )
            rosType="desktop-full"
            menu="0";;
    esac
done

sudo apt install ros-$rosVersion-$rosType -y


# Sourcing.
echo "# ROS SOURCING" >> ~/.bashrc
echo "source /opt/ros/$rosVersion/setup.bash" >> ~/.bashrc
source ~/.bashrc


# Initializing.
case $rosVersion in
    "melodic" )
        sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
        sudo apt install python-rosdep;;
    "noetic" )
        sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
        sudo apt install python3-rosdep;;
esac

sudo rosdep init
rosdep update

# Return to original directory.
cd $curDirROS