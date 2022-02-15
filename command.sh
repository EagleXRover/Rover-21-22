#!/bin/bash

case $(lsb_release -r -s) in
    "20.04" )
        rosVersion="noetic";;
    "18.04" )
        rosVersion="melodic";;
esac

menu="1"
while [[ $menu == "1" ]]
do 
    echo " "
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

echo $rosVersion
echo $rosType