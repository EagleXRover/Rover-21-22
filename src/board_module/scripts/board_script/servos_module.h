/*
  servos_module.h - Library for the control of the servos .
  Created by CatBookshelf, April 5, 2022.
*/

#ifndef SERVOS_MODULE_H
#define SERVOS_MODULE_H

#include <Arduino.h>
#include "minimal_module.h"
#include <Servo.h>

#include <std_msgs/UInt8.h>

void setupServos(void);

void servoArmWristPitchCb( const std_msgs::UInt8&);
void servoArmGripperCb( const std_msgs::UInt8&);
void servoScienceMicroscopeCb( const std_msgs::UInt8&);
void servoScienceDispenserExteriorCb( const std_msgs::UInt8&);
void servoScienceDispenserInteriorCb( const std_msgs::UInt8&);

// Servos pins.
#define Servo_arm_wrist_pitch           2   // Servo control pin for arm wrist pitch.
#define Servo_arm_gripper_claw          3   // Servo control pin for arm gripper claw.
#define Servo_science_microscope        4   // Servo control pin for science microscope.
#define Servo_science_dispencer_extern  5   // Servo control pin for extern dispencer.
#define Servo_science_dispencer_intern  6   // Servo control pin for intern dispencer.

// Servos objects.
Servo Servo_arm_wrist_pitch_obj;
Servo Servo_arm_gripper_claw_obj;
Servo Servo_science_microscope_ob;
Servo Servo_science_dispencer_extern_obj;
Servo Servo_science_dispencer_intern_obj;

// ROS constants.
#define topic_servo_arm_wristPitch              "/arm/servos/wrist"
#define topic_servo_arm_gripper                 "/arm/servos/gripper"
// #define topic_motors_science                    "/science/drivers"
#define topic_servo_science_microscope          "/science/servos/microscope"
#define topic_servo_science_dispenser_exterior  "/science/servos/dispenser/exterior"
#define topic_servo_science_dispenser_interior  "/science/servos/dispenser/interior"

// ROS Subscribers.
ros::Subscriber<std_msgs::UInt8> sub_servo_arm_wristPitch (topic_servo_arm_wristPitch, &servoArmWristPitchCb);
ros::Subscriber<std_msgs::UInt8> sub_servo_arm_gripper (topic_servo_arm_gripper, &servoArmGripperCb);
//ros::Subscriber<std_msgs::UInt8> sub_motors_science (topic_motors_science, &motorsScienceCb);
ros::Subscriber<std_msgs::UInt8> sub_servo_science_microscope (topic_servo_science_microscope, &servoScienceMicroscopeCb);
ros::Subscriber<std_msgs::UInt8> sub_servo_science_dispenser_exterior (topic_servo_science_dispenser_exterior, &servoScienceDispenserExteriorCb);
ros::Subscriber<std_msgs::UInt8> sub_servo_science_dispenser_interior (topic_servo_science_dispenser_interior, &servoScienceDispenserInteriorCb);

void setupServos(void){
    Servo_arm_wrist_pitch_obj.attach(Servo_arm_wrist_pitch);
    Servo_arm_gripper_claw_obj.attach(Servo_arm_gripper_claw);
    Servo_science_microscope_ob.attach(Servo_science_microscope);
    Servo_science_dispencer_extern_obj.attach(Servo_science_dispencer_extern);
    Servo_science_dispencer_intern_obj.attach(Servo_science_dispencer_intern);

    nh.subscribe(sub_servo_arm_wristPitch);
    nh.subscribe(sub_servo_arm_gripper);
    // nh.subscribe(sub_motors_science);
    nh.subscribe(sub_servo_science_microscope);
    nh.subscribe(sub_servo_science_dispenser_exterior);
    nh.subscribe(sub_servo_science_dispenser_interior);
}

void servoArmWristPitchCb( const std_msgs::UInt8 &msg){
    Servo_arm_wrist_pitch_obj.write(msg.data);
}

void servoArmGripperCb( const std_msgs::UInt8 &msg){
    Servo_arm_gripper_claw_obj.write(msg.data);
}

// // Callback function for Science motors.
// void motorsScienceCb( const std_msgs::UInt8 &msg){
//     uint8_t data = msg.data;
//     for (uint8_t i = 0; i < Motors_Science_Amount; i++){
//         if (data & (0x02 << (Motors_Science_Amount - 1 - i))){
//             if (data & (0x01 << (Motors_Science_Amount - 1 - i)))
//                 RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed + Motors_ScienceFast);
//             else 
//                 RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed - Motors_ScienceSlow);
//         } else
//             RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed);
//     }
// }

void servoScienceMicroscopeCb( const std_msgs::UInt8 &msg){
    Servo_science_microscope_ob.write(msg.data);
}

void servoScienceDispenserExteriorCb( const std_msgs::UInt8 &msg){
    Servo_science_dispencer_extern_obj.write(msg.data);
}

void servoScienceDispenserInteriorCb( const std_msgs::UInt8 &msg){
    Servo_science_dispencer_intern_obj.write(msg.data);
}

#endif