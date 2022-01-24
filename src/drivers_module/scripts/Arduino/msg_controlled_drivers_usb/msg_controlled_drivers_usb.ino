/* 
  msg_controlled_drivers_usb
*/

#include "RoboClaw.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>

/* Constants declarations. */
// Pins connections.
#define led_pin             0x0D
#define arduinoTx_S1_wheels 0x0C
#define arduinoTx_S1_arm    0x0B
#define arduinoRx_S2_arm    0x0A
#define arduinoRx_S2_wheels 0x09

// Wheels driver setup and data.
#define speed_shiftLeft_fwd 0x05
#define speed_shiftLeft_bwd 0x04
const int left_wheels[] = {0x80, 0x82, 0x84};
const int right_wheels[] = {0x81, 0x83, 0x85};
const unsigned int wheels_driver_baudrate = 38400;
SoftwareSerial serial_wheels(arduinoRx_S2_wheels,arduinoTx_S1_wheels);
RoboClaw driver_wheels(&serial_wheels,10000);

// Arm drivers setup and data.
#define arm_linear          0x80
#define arm_rotational      0x81
#define wrist_rotational    0x82
#define zero_speed          0x00
#define linear_speed        0x40
#define rot_speed           0x20
const unsigned int arm_driver_baudrate = 38400;
SoftwareSerial serial_arm(arduinoRx_S2_arm,arduinoTx_S1_arm);
RoboClaw driver_arm(&serial_arm,10000);

/* Global variables / flags */
char *wheels_speed = new char(0x00); 
unsigned long arduino_timer = 0;
unsigned long ros_timer = 0;

/* Functions declarations. */
// ROS Subscribers functions.
void armSubscriber(const std_msgs::Int16 arm);
void wheelsSubscriber(const std_msgs::Int16 wheels);
void timeOutSubscriber(const std_msgs::Empty update);

// Halt funcitons.
void haltMovement(void);
void stopArm(void);
void stopWheels(void);

// Setup functions.
void setup(void);

/* ROS setup */
ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Int16> arm_sub("arm_topic", &armSubscriber);
ros::Subscriber<std_msgs::Int16> wheels_sub("wheels_topic", &wheelsSubscriber);
ros::Subscriber<std_msgs::Empty> timeOut_sub("timeout_topic", &timeOutSubscriber );

/* Function definitions. */
// Arm controlling function.
void armSubscriber(const std_msgs::Int16 arm){
  char aux = (arm.data>>(3*2)) & 0x03;
  if (aux & 0x02){
    if (aux &0x01) driver_arm.BackwardM2(arm_rotational,rot_speed);
    else driver_arm.ForwardM2(arm_rotational,rot_speed);
  }
  else driver_arm.ForwardM2(arm_rotational,0);

  aux = (arm.data>>(2*2)) & 0x03;
  if (aux & 0x02){
    if (aux &0x01) driver_arm.BackwardM1(arm_linear,linear_speed);
    else driver_arm.ForwardM1(arm_linear,linear_speed);
  }
  else driver_arm.ForwardM1(arm_linear,0);

  aux = (arm.data>>(1*2)) & 0x03;
  if (aux & 0x02){
    if (aux &0x01) driver_arm.BackwardM2(arm_linear,linear_speed);
    else driver_arm.ForwardM2(arm_linear,linear_speed);
  }
  else driver_arm.ForwardM2(arm_linear,0);

  aux = (arm.data>>(0*2)) & 0x03;
  if (aux & 0x02){
    if (aux &0x01) driver_arm.ForwardM1(wrist_rotational,rot_speed);
    else driver_arm.BackwardM1(wrist_rotational,rot_speed);
  }
  else driver_arm.ForwardM1(wrist_rotational,0);
  //delay(1);
}

// Wheels controlling function.
void wheelsSubscriber(const std_msgs::Int16 wheels){
  //if (wheels.data != wheels_speed){
  char aux1 = wheels.data & 0xF0;

  if (aux1 & 0x80){
    aux1 = aux1 ^ 0x80;
    aux1 = aux1 >> 1;
    driver_wheels.BackwardM1(left_wheels[0],aux1);
    driver_wheels.BackwardM1(left_wheels[1],aux1);
    driver_wheels.BackwardM1(left_wheels[2],aux1);
  } else {
    driver_wheels.ForwardM1(left_wheels[0],aux1);
    driver_wheels.ForwardM1(left_wheels[1],aux1);
    driver_wheels.ForwardM1(left_wheels[2],aux1);
  }

  aux1 = (wheels.data << 4) & 0xF0;
  
  if (aux1 & 0x80){
    aux1 = aux1 ^ 0x80;
    aux1 = aux1 >> 1;
    driver_wheels.BackwardM1(right_wheels[0],aux1);
    driver_wheels.BackwardM1(right_wheels[1],aux1);
    driver_wheels.BackwardM1(right_wheels[2],aux1);
  } else {
    driver_wheels.ForwardM1(right_wheels[0],aux1);
    driver_wheels.ForwardM1(right_wheels[1],aux1);
    driver_wheels.ForwardM1(right_wheels[2],aux1);
  }

}

void timeOutSubscriber(const std_msgs::Empty update){
  ros_timer = millis();
}

void setup()
{ 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(led_pin, LOW);

  driver_arm.begin(arm_driver_baudrate);
  driver_wheels.begin(wheels_driver_baudrate);


  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(wheels_sub);
  nh.subscribe(timeOut_sub);
  arduino_timer = millis();
  ros_timer = millis();
}

void loop()
{ 
  arduino_timer = millis();
  if ((arduino_timer - ros_timer) > 500) haltMovement();
  nh.spinOnce();
  //delay(1);
}

void haltMovement(void){
  stopArm();
  stopWheels();
}

void stopArm(void){
  driver_arm.ForwardM1(arm_linear,zero_speed);
  driver_arm.ForwardM2(arm_linear,zero_speed);
  driver_arm.ForwardM2(arm_rotational,zero_speed);
  driver_arm.ForwardM1(wrist_rotational,zero_speed);
}

void stopWheels(void){
  for (char i = 0; i < sizeof(left_wheels); i++){
    driver_wheels.ForwardM1(left_wheels[i],zero_speed);
  }
  for (char i = 0; i < sizeof(left_wheels); i++){
    driver_wheels.ForwardM1(right_wheels[i],zero_speed);
  }
}
