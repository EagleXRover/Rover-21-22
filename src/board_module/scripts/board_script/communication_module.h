/*
  communication_module.h - Library for the setup of the communication
  on the board.
  Created by CatBookshelf, April 1, 2022.
  Last updated, September 13, 2022.
*/

#ifndef COMMUNICATION_MODULE_H
#define COMMUNICATION_MODULE_H

// Include basic libraries 
#include <Arduino.h>
#include "minimal_module.h"
#include "constants_module.h"



// Include and define Ethernet parameters 
#define ETHERNET_CS         53
#define ETHERNET_RST        48
byte mac[] = {
    0x80, 0x69, 0x69, 0x69, 0x69, 0x09
};
#include <SPI.h>
#include <UIPEthernet.h>



// Include ROS
#define ROSSERIAL_ARDUINO_TCP
#define ROS_TIMEOUT     3500 // ms 
#define topic_watchdog  "/watchdog_topic"
unsigned long watchPrevTime;
#include <ros.h>
#include <std_msgs/Empty.h>



// Declare functions
void watchdogUpdater(void);
void ros_setup(void);
void ethernet_setup(void);
void comms_update(void);
void comms_setup(void);



// Initialize ROS parameters
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Empty> sub_watchdog (topic_watchdog, &watchdogUpdater);
IPAddress server(10, 0, 0, 4);
uint16_t serverPort = 11411;




// Define functions

// watchdog function updater
void watchdogUpdater(void){
    watchPrevTime = millis();
}

void ros_setup(void){
    nh.getHardware() -> setConnection(server, serverPort);
    nh.initNode();
    nh.subscribe(sub_watchdog);
}

void ethernet_setup(void){
  pinMode(ETHERNET_RST, OUTPUT);
  digitalWrite(ETHERNET_RST, LOW);
  delay(10);
  digitalWrite(ETHERNET_RST, HIGH);

  Ethernet.init(ETHERNET_CS);

  delay(100);

  if (DEBUG_ON) 
    Serial_USB.println("\nInitialize Ethernet : ");

  if (Ethernet.begin(mac) == 0){
    if (DEBUG_ON){
      Serial_USB.println("Failed to get IP address...");
      if (Ethernet.hardwareStatus() == EthernetNoHardware)
        Serial_USB.println("Ethernet shield was not found.");
      else if (Ethernet.linkStatus() == LinkOFF)
        Serial_USB.println("Ethernet cable is not connected.");
      // No point on continuing, so restart and attempt again.
      delay(100);
    }
    reboot("Failed to setup Ethernet...");
    
  }
  
  digitalWrite(LED_NOTIFICATION, LOW);
  Serial_USB.print("My IP address is : ");
  Serial_USB.println(Ethernet.localIP());

}

void comms_update(void){
  actualTime = millis();
  if ((actualTime - watchPrevTime) > ROS_TIMEOUT){
    Serial.println(actualTime);
    Serial.println(watchPrevTime);
    reboot("ROS Timeout exceeded...");
  }
  if ( Ethernet.maintain()%2 || 
    Ethernet.linkStatus() == LinkOFF
  )
    reboot("Ethernet exception...");
  nh.spinOnce();
}

void comms_setup(void){
  ethernet_setup();
  ros_setup();
  watchPrevTime = millis();
  comms_update();
  delay(1);
}


#endif
