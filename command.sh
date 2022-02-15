#!/bin/bash

joySetup(){
    echo "Hello World!"
}

DIR="$(pwd)"
if [ !-d "~/catkin_ws/src"]; then
   echo "WITHOUT CATKIN WORKSPACE"
else
   echo "CATKIN WORKSPACE AVAILABLE"
fi

#menu="1"
#while [[ $menu == "1" ]]
#do
#    read -p "Print? [Y/N] : " Selection
#    case ${Selection,,} in
#        "y" )
#            joySetup
#            menu="0";;
#        "n" )
#            menu="0";;
#    esac
#done
#
#[ -d "~/catkin_ws" ] && echo "Directory ~/catkin_ws exists."
#
#
#if [[ -d "~/catkin_ws" ]]; then echo "hi"; else echo "ih"; fi