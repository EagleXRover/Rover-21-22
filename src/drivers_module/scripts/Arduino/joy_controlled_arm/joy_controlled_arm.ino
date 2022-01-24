/* 
 joy_controlled_arm
 This sketch allows to use an arduino as a driver's control unit, 
 while implementing ROS for the remote control via a joy controller.

 It requires to have a topic with sensor_msgs/Joy type of data, but
 it enables to get the data and transmit it to the roboclaw drivers 
 via serial communication.
*/

#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);  
RoboClaw roboclaw(&serial,10000);

#define arm_linear       0x80
#define arm_rotational   0x81
#define wrist_rotational 0x82
#define speed_val        0x10

ros::NodeHandle  nh;

void messageCb(const sensor_msgs::Joy& joy_msg){
  float* axes;
  long int* buttons;
  int* aux_flag = new int;
  axes = &(joy_msg.axes[0]);
  buttons = &(joy_msg.buttons[0]);

  if (buttons[6] != 0){
    if (axes[5] <= 0){
      roboclaw.ForwardBackwardM1(arm_linear,0x40 + 2*speed_val);
    } else if(buttons[5] != 0){
      roboclaw.ForwardBackwardM1(arm_linear,0x40 - 2*speed_val);
    } else {
      roboclaw.ForwardBackwardM1(arm_linear,0x40);
    }
  
    if (buttons[4] != 0){
      roboclaw.ForwardBackwardM2(arm_linear,0x40 + 2*speed_val);
    } else if(axes[2] <= 0){
      roboclaw.ForwardBackwardM2(arm_linear,0x40 - 2*speed_val);
    } else {
      roboclaw.ForwardBackwardM2(arm_linear,0x40);
    }
  
    if (axes[6] > .25){
      roboclaw.ForwardBackwardM1(wrist_rotational,0x40 + speed_val);
    } else if(axes[6] < -.25){
      roboclaw.ForwardBackwardM1(wrist_rotational,0x40 - speed_val);
    } else {
      roboclaw.ForwardBackwardM1(wrist_rotational,0x40);
    }
  
    if (buttons[1] > 0){
      roboclaw.ForwardBackwardM2(arm_rotational,0x40 + speed_val);
    } else if(buttons[2] > 0){
      roboclaw.ForwardBackwardM2(arm_rotational,0x40 - speed_val);
    } else {
      roboclaw.ForwardBackwardM2(arm_rotational,0x40);
    }
  } else {
    roboclaw.ForwardBackwardM1(arm_linear,0x40);
    roboclaw.ForwardBackwardM2(arm_linear,0x40);
    roboclaw.ForwardBackwardM1(wrist_rotational,0x40);
    roboclaw.ForwardBackwardM2(arm_rotational,0x40);
  }

  delete aux_flag;
  aux_flag = NULL;
  axes = NULL;
  buttons = NULL;
}

ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);
  roboclaw.begin(38400);
}

void loop()
{  
  delay(1);
  nh.spinOnce();
  delay(1);
}
