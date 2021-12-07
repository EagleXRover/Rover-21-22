/*
 msg_controlled_arm
*/


/* Libraries import. */
// General libraries.
#include <SPI.h>
#include <Ethernet.h>
#include "RoboClaw.h"

// To use the TCP version of rosserial_arduino.
#define ROSSERIAL_ARDUINO_TCP

// Ros packages and message libraries.
#include <ros.h>
#include <std_msgs/Byte.h>

/* Constants declarations. */
// Pins connections.
#define led_pin             0x0D
#define arduinoTx_S1        0x0B
#define arduinoRx_S2        0x0A
#define shield_CS           0x09

// Arm drivers setup and data.
#define arm_linear          0x80
#define arm_rotational      0x81
#define wrist_rotational    0x82
#define zero_speed          0x00
#define linear_speed        0x40
#define rot_speed           0x20
const unsigned int driver_baudrate = 38400;
SoftwareSerial serial(arduinoRx_S2,arduinoTx_S1);
RoboClaw driver_arm(&serial,10000);

// Notification sequences freq.
#define notification_time   2000 //ms has to at least be 1s long
#define note_eighths        0x08
#define note_quarters       0x04

// Shield settings data.
byte mac[] = { 0xC3, 0xEB, 0x7D, 0xC2, 0x21, 0x5F };
IPAddress slave_ip(10, 0, 0, 11);

// Master settings data.
IPAddress master_ip(10, 0, 0, 4);
uint16_t master_port = 11411;

/* Global variables / flags */
bool *error = new bool(false);

/* Functions declarations. */
// ROS Subscribers functions.
void armSubscriber(const std_msgs::Byte arm);

// Setup functions.
void setup(void);
void shieldSetup(void);

// Loop / idle function.
void loop(void);

// Error functions.
void haltIfError(void);
void haltIfNotHardware(bool* error);
void haltIfNotLinked(bool* error);
void haltMovement(void);

// Stopping movements of drivers.
void stopArm(void);

// Notificiation funcions.
void blink(int freq);
void blinkNoShield(void);
void blinkNoLink(void);
void blinkConnected(void);

/* ROS setup */
ros::NodeHandle arm_nodeHandler;
ros::Subscriber<std_msgs::Byte> arm_sub("arm_topic", &armSubscriber);

/* Function definitions. */
// Arm controlling function.
void armSubscriber(const std_msgs::Byte arm){
  char aux = (arm.data>>(3*2)) & 0x03;
  if (aux & 0x02){
    if (aux &0x01) driver_arm.ForwardM2(arm_rotational,rot_speed);
    else driver_arm.BackwardM2(arm_rotational,rot_speed);
  }
  aux = (arm.data>>(2*2)) & 0x03;
  if (aux & 0x02){
    if (aux &0x01) driver_arm.ForwardM1(arm_linear,linear_speed);
    else driver_arm.BackwardM1(arm_linear,linear_speed);
  }
  aux = (arm.data>>(1*2)) & 0x03;
  if (aux & 0x02){
    if (aux &0x01) driver_arm.ForwardM2(arm_linear,linear_speed);
    else driver_arm.BackwardM2(arm_linear,linear_speed);
  }
  aux = (arm.data>>(0*2)) & 0x03;
  if (aux & 0x02){
    if (aux &0x01) driver_arm.ForwardM1(wrist_rotational,rot_speed);
    else driver_arm.BackwardM1(wrist_rotational,rot_speed);
  }
  //delay(1);
}

// Setup function for arduino chip.
void setup(void) { 
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  driver_arm.begin(driver_baudrate);

  shieldSetup();
  
  arm_nodeHandler.getHardware() -> setConnection(master_ip, master_port);
  arm_nodeHandler.initNode();
  arm_nodeHandler.subscribe(arm_sub);
  
}

// Loop function.
void loop(void) { 
  haltIfError();
  arm_nodeHandler.spinOnce();
}

// Setup function for the Ethernet Shield.
void shieldSetup(void){ 
  Ethernet.init(shield_CS); // setup CS pin for the shield.
  *error = false;
  haltIfError();
  //successBlink();
}

// Halts if there is an error.
void haltIfError(void){
  *error = false;
  haltIfNotHardware(error);
  haltIfNotLinked(error);
  if (*error = true){
    *error = false;
    blinkConnected();
  }
}

// Halts if there is an error with the shield connection.
void haltIfNotHardware(bool* error){
  while (Ethernet.hardwareStatus() == EthernetNoHardware){
    if (*error == false){
      *error = true;
      haltMovement();
    }
    Ethernet.begin(mac,slave_ip);
    blinkNoShield();
  }
}

// Halts if there is an error with the link connection.
void haltIfNotLinked(bool* error){
  while (Ethernet.linkStatus() != LinkON){
    if (*error == false){
      *error = true;
      haltMovement();
    }
    void blinkNoLink(void);
  }
}

// Halts movements in case of connection error.
void haltMovement(void) {
  stopArm();
}

// Stops movement of the arm.
void stopArm(void){
  driver_arm.ForwardM1(arm_linear,zero_speed);
  driver_arm.ForwardM2(arm_linear,zero_speed);
  driver_arm.ForwardM2(arm_rotational,zero_speed);
  driver_arm.ForwardM1(wrist_rotational,zero_speed);
}


// Blink dessired LED at a desired frequency.
void blink(int freq){ 
  digitalWrite(led_pin, 1^digitalRead(led_pin));
  delay(notification_time/2/freq);
  digitalWrite(led_pin, 1^digitalRead(led_pin));
  delay(notification_time/2/freq);
}

// Blink sequence for no hardware detected.
void blinkNoShield(void){
  for (char i=0; i<2; i++) blink(note_eighths);
  for (char i=0; i<2; i++) blink(note_quarters);
  delay(notification_time/note_quarters);
}

// Blink sequence for no hardware detected.
void blinkNoLink(void){
  for (char i=0; i<2; i++){
    blink(note_eighths);
    blink(note_quarters);
  }
  delay(notification_time/note_quarters);
}

// Blink sequence for no hardware detected.
void blinkConnected(void){
  for (char i=0; i<4; i++){
    blink(note_quarters);
  }
}
