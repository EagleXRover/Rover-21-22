# joy_controlled_arm.ino
This script works by using the roboclaw library for arduino, in which multiples addresses were used for a proper control.

The assigned addresses for the arm were 0x80, for the control of the extension of the motors for the pitch and height of the robotic arm, which allowed the arm to move up and down; the address 0x81 was assigned as the rotation of the whole arm allowing which allowed the movement from left to right at the base of the arm; lastly, the address 0x82 was assigned as the wrist rotation control, which allowed the movement at clockwise and counter clockwise turning of the hand / gripper. 

There was a mix of both solo and dual drivers used at the same time, which made it possible for it to use in some cases the motor 1 and 2 of the assigned addresses, while others only used the motor 1 of said address.

## Control:

For this script a Xbox One control was used as an input device for the manipulation of the robotic arm, in which a series of buttons and axis inputs were used as triggers for the movements.

* RB / RT : Used as the triggers for height / pitch for the *upper arm* part of the robotic arm.
* LB / LT : Used as the triggers of the height / pitch for the *forearm* part of the robotic arm.
* Left / Right D-Pad buttons : Used as the trigger buttons for the rotation of the wrist of the robotic arm.
* X / B : Used as the trigger buttons for the rotation of the whole arm as left and right.


## Requires:
- ROS
- rosduino
- joy package
- rosserial
- roboclaw arduino library

## Hardware used:
- Arduino Mega
- Roboclaw solo motor controller
- Roboclaw dual motor controller
- Raspberry Pi 4B

## Running:
For the script to be able to communicate between the Raspberry Pi and the Arduino we have to execute the next command:

```
rosrun rosserial_python serial_node.py /dev/ttyACMX
```
You need to take into account that the port may be different in your case, and you have to specify the correct one instead of the "*/dev/ttyACMX*" that we used on the command above for the correct setup. 