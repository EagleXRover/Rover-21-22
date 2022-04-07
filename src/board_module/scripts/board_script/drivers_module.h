/*
  drivers_module.h - Library for the control of the 
  Roboclaw drivers for the wheels and arm.
  Created by CatBookshelf, April 1, 2022.
*/

#ifndef DRIVERS_MODULE_H
#define DRIVERS_MODULE_H

#include <Arduino.h>
#include "minimal_module.h"
#include "RoboClaw.h"

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>

void setupDrivers(void);

void motorsWheelsCb( const std_msgs::UInt16&);
void motorsArmCb( const std_msgs::UInt8&);
void motorsScienceCb( const std_msgs::UInt8&);

void haltMovements(void);

#define reboot reboot2
void reboot2(String message);


#define Timeout_Wheels      10000       // Timeout for Wheels drivers.
#define Timeout_Arm_Science 10000       // Timeout for Arm & Science drivers.

// Motor constants.
#define Motors_HaltSpeed 64
#define Motors_ScienceSlow 10
#define Motors_ScienceFast 63
#define Motors_MovementSpeedDiff 0x10
#define Motors_Wheels_Amount_PerSide 3
#define Motors_Arm_Amount 4
#define Motors_Science_Amount 1
const uint8_t Motors_Wheels_Left[Motors_Wheels_Amount_PerSide] = {      // Wheels Left motors array
    0x80, 0x82, 0x84
};
const uint8_t Motors_Wheels_Right[Motors_Wheels_Amount_PerSide] = {     // Wheels Right motors arra
    0x81, 0x83, 0x85
};
const uint8_t Motors_Arm[Motors_Arm_Amount] = {                         // Arm motors array
    0x81, 0x82, 0x80, 0x83
};
const uint8_t Motors_Science[Motors_Science_Amount] = {                 // Science motors array
    0x84
};

// Drivers constants / definitions.
RoboClaw RoboClaw_Wheels = RoboClaw(&Serial_Wheels, Timeout_Wheels);                    // Roboclaw Wheels driver
RoboClaw RoboClaw_Arm_Science = RoboClaw(&Serial_Arm_Science, Timeout_Arm_Science);     // Roboclaw Arm & Science driver

// ROS constants.
#define topic_motors_wheels                     "/wheels/drivers"
#define topic_motors_arm                        "/arm/drivers"
#define topic_motors_science                    "/science/drivers"

// ROS Subscribers.
ros::Subscriber<std_msgs::UInt16> sub_motors_wheels(topic_motors_wheels, &motorsWheelsCb);
ros::Subscriber<std_msgs::UInt8> sub_motors_arm(topic_motors_arm, &motorsArmCb);
ros::Subscriber<std_msgs::UInt8> sub_motors_science (topic_motors_science, &motorsScienceCb);

// Flags.
bool driversReady = false;

void setupDrivers(void){
    RoboClaw_Wheels.begin(Baudrate_Wheels);
    RoboClaw_Arm_Science.begin(Baudrate_Arm_Science);
    driversReady = true;
    nh.subscribe(sub_motors_wheels);
    nh.subscribe(sub_motors_arm);
    nh.subscribe(sub_motors_science);
}

// Callback function for wheels motors.
void motorsWheelsCb( const std_msgs::UInt16 &msg){
    int left_speed = msg.data >> 8;
    int right_speed = msg.data & 0x00FF;
    for(uint8_t i = 0; i < Motors_Wheels_Amount_PerSide; i++){
        RoboClaw_Wheels.ForwardBackwardM1(Motors_Wheels_Left[i], left_speed);
        RoboClaw_Wheels.ForwardBackwardM1(Motors_Wheels_Right[i], right_speed);
    }
}

// Callback function for arm motors.
void motorsArmCb( const std_msgs::UInt8 &msg){
    uint8_t data = msg.data;
    for (uint8_t i = 0; i < Motors_Arm_Amount; i++){
        if (data & 0x80){
            if (data & 0x40)
                RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Arm[i], Motors_HaltSpeed + Motors_MovementSpeedDiff);
            else 
                RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Arm[i], Motors_HaltSpeed - Motors_MovementSpeedDiff);
        } else 
            RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Arm[i], Motors_HaltSpeed);
        data = data << 2;
    }
}

// Callback function for Science motors.
void motorsScienceCb( const std_msgs::UInt8 &msg){
    uint8_t data = msg.data;
    for (uint8_t i = 0; i < Motors_Science_Amount; i++){
        if (data & (0x02 << (Motors_Science_Amount - 1 - i))){
            if (data & (0x01 << (Motors_Science_Amount - 1 - i)))
                RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed + Motors_ScienceFast);
            else 
                RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed - Motors_ScienceSlow);
        } else
            RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed);
    }
}

// Stop Movements
void haltMovements(void){
    if (!driversReady) return;
    for (int i = 0; i < Motors_Wheels_Amount_PerSide; i++){
        RoboClaw_Wheels.ForwardBackwardM1(Motors_Wheels_Left[i],Motors_HaltSpeed);
        RoboClaw_Wheels.ForwardBackwardM1(Motors_Wheels_Right[i],Motors_HaltSpeed);
    }
    for (int i = 0; i < Motors_Arm_Amount; i++)
        RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Arm[i], Motors_HaltSpeed);
    for (int i = 0; i < Motors_Science_Amount; i++)
        RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed);
}

// Reboots the system.
void reboot2(String message){
    Serial_USB.println(message);
    Serial_USB.println("REBOOTING...");
    haltMovements();
    delay(100);
    reboot_funct();
}
#endif