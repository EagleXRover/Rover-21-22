#ifndef BASE_MODULE_BASE_NODE_H
#define BASE_MODULE_BASE_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"

#define scale_wheelsSpeed 0.5


#define LEFT_JOYSTICK_H     axes[0]
#define LEFT_JOYSTICK_V     axes[1]
#define LEFT_TRIGGER        axes[2]
#define RIGHT_JOYSTICK_H    axes[3]
#define RIGHT_JOYSTICK_V    axes[4]
#define RIGHT_TRIGGER       axes[5]
#define D_PAD_H             axes[6]
#define D_PAD_V             axes[7]
#define BUTTON_A            buttons[0]
#define BUTTON_B            buttons[1]
#define BUTTON_X            buttons[2]
#define BUTTON_Y            buttons[3]
#define BUTTON_LB           buttons[4]
#define BUTTON_RB           buttons[5]
#define BUTTON_BACK         buttons[6]
#define BUTTON_PLAY         buttons[7]
#define BUTTON_GUIDE        buttons[8]
#define BUTTON_L3           buttons[9]
#define BUTTON_R3           buttons[10]



// #define topic_motors_wheelsLeft                 "/wheels/drivers/left"
// #define topic_motors_wheelsRight                "/wheels/drivers/right"
#define topic_motors_wheels                     "/wheels/drivers"
// #define topic_motors_armScience                 "/arm_science/drivers"
#define topic_motors_arm                        "/arm/drivers"
#define topic_servo_arm_wristPitch              "/arm/servos/wrist"
#define topic_servo_arm_gripper                 "/arm/servos/gripper"
#define topic_motors_science                    "/science/drivers"
#define topic_servo_science_microscope          "/science/servos/microscope"
#define topic_servo_science_dispenser_exterior  "/science/servos/dispenser/exterior"
#define topic_servo_science_dispenser_interior  "/science/servos/dispenser/interior"

#define topic_queue_size    1

const uint8_t limit_servo_arm_wristpitch[2] = {30, 90};
const uint8_t limit_servo_arm_gripper[2] = {45, 135};
const uint8_t limit_servo_science_microscope[2] = {0, 180};
const uint8_t limit_servo_science_dispenser_exterior[2] = {5, 175};
const uint8_t limit_servo_science_dispenser_interior[2] = {5, 175};

const uint8_t speed_servo_arm_wristpitch = 5;
const uint8_t speed_servo_arm_gripper = 5;
const uint8_t speed_servo_science_microscope = 5;
const uint8_t speed_servo_science_dispenser_exterior = 178;
const uint8_t speed_servo_science_dispenser_interior = 178;

class BaseNode{
    protected:
        ros::NodeHandle *nh;
        ros::Subscriber joy;
        ros::Publisher 
            // motors_wheelsLeft, 
            // motors_wheelsRight, 
            motors_wheels,
            // motors_armScience, 
            motors_arm,
            motors_science,
            servo_arm_wristPitch, 
            servo_arm_gripper, 
            servo_science_microscope, 
            servo_science_dispenser_exterior, 
            servo_science_dispenser_interior;
        uint8_t 
            mov_motors_Arm = 0x00,
            mov_motors_Science = 0x00,
            pos_servo_arm_wristPitch = 50, 
            pos_servo_arm_gripper = 90, 
            pos_servo_science_microscope = 90, 
            pos_servo_science_dispenser_exterior = 2, 
            pos_servo_science_dispenser_interior = 2;
        uint16_t speed_motors_Wheels = 0x4040;
    
    public:
        BaseNode(ros::NodeHandle *nh, const char subTopic[]);
        void callback (const sensor_msgs::Joy::ConstPtr&msg);
        void publish(void);
};

BaseNode::BaseNode(ros::NodeHandle *nh, const char subTopic[]):
    nh(nh){
    joy = nh -> subscribe(subTopic, topic_queue_size, &BaseNode::callback, this);
    motors_wheels = nh -> advertise<std_msgs::UInt16>(topic_motors_wheels, topic_queue_size, true);
    motors_arm = nh -> advertise<std_msgs::UInt8>(topic_motors_arm, topic_queue_size, true);
    motors_science = nh -> advertise<std_msgs::UInt8>(topic_motors_science, topic_queue_size, true);
    servo_arm_wristPitch = nh -> advertise<std_msgs::UInt8>(topic_servo_arm_wristPitch, topic_queue_size, true);
    servo_arm_gripper = nh -> advertise<std_msgs::UInt8>(topic_servo_arm_gripper, topic_queue_size, true);
    servo_science_microscope = nh -> advertise<std_msgs::UInt8>(topic_servo_science_microscope, topic_queue_size, true);
    servo_science_dispenser_exterior = nh -> advertise<std_msgs::UInt8>(topic_servo_science_dispenser_exterior, topic_queue_size, true);
    servo_science_dispenser_interior = nh -> advertise<std_msgs::UInt8>(topic_servo_science_dispenser_interior, topic_queue_size, true);
}

void BaseNode::callback(const sensor_msgs::Joy::ConstPtr&msg){
    uint16_t aux_uint16;
    uint8_t aux_uint8;
    std_msgs::UInt16 aux_msg_uint16;
    std_msgs::UInt8 aux_msg_uint8;

    /* Wheel control */
    // Left joystick for wheels movements 
    float x = -msg->axes[0];
    float y = msg->axes[1];
    aux_uint16 = 0x4040; 
    aux_uint16 += uint16_t(0x3F * scale_wheelsSpeed * (x+y) * 0.7071067811865) << 8;
    aux_uint16 += uint16_t(0x3F * scale_wheelsSpeed * (-x+y) * 0.7071067811865);
    if (speed_motors_Wheels != aux_uint16){
        speed_motors_Wheels = aux_uint16;
        aux_msg_uint16.data = speed_motors_Wheels;
        motors_wheels.publish(aux_msg_uint16);
    }

    /* Arm control */
    // D-Pad Left and Right for shoulder arm yaw control ->
    aux_uint8 = 0;
    if (msg -> D_PAD_H > 0.25 || msg -> D_PAD_H < -0.25) 
        aux_uint8 += 0x02;
    aux_uint8 += (msg -> D_PAD_H > 0.25);
    
    // D-Pad Up and Down for shoulder pitch control ->
    aux_uint8 = aux_uint8 << 2;
    if (msg -> D_PAD_V > 0.25 || msg -> D_PAD_V < -0.25)
        aux_uint8 += 0x02;
    aux_uint8 += (msg -> D_PAD_V > 0.25);
    
    // LB and RB for elbow pitch control ->
    aux_uint8 = aux_uint8 << 2;
    if (msg -> BUTTON_LB || msg -> BUTTON_RB) 
        aux_uint8 += 0x02;
    aux_uint8 += (msg -> BUTTON_LB);

    // Horizontal right joystick Left and Right for wrist roll ->
    aux_uint8 = aux_uint8 << 2;
    if (msg -> RIGHT_JOYSTICK_H > 0.25 || msg -> RIGHT_JOYSTICK_H < -0.25)
        aux_uint8 +=0x02;
    aux_uint8 += (msg -> RIGHT_JOYSTICK_H > 0.25);

    if (mov_motors_Arm != aux_uint8){
        mov_motors_Arm = aux_uint8;
        aux_msg_uint8.data = mov_motors_Arm;
        motors_arm.publish(aux_msg_uint8);
    }

    // X and Y for centrifuge motor control ->
    aux_uint8 = 0;
    if (msg -> BUTTON_X || msg -> BUTTON_Y)
        aux_uint8 += 0x02;
    aux_uint8 += (msg -> BUTTON_X);

    if (mov_motors_Science != aux_uint8){
        mov_motors_Science = aux_uint8;
        aux_msg_uint8.data = mov_motors_Science;
        motors_science.publish(aux_msg_uint8);
    }

    // Vertical right Joytsick Up and Down for wrist pitch -> 
    aux_uint8 = pos_servo_arm_wristPitch;
    if ((msg -> RIGHT_JOYSTICK_V > 0.25) && (pos_servo_arm_wristPitch < limit_servo_arm_wristpitch[1]))
        aux_uint8 += speed_servo_arm_wristpitch;
    if ((msg -> RIGHT_JOYSTICK_V < -0.25) && (pos_servo_arm_wristPitch > limit_servo_arm_wristpitch[0]))
        aux_uint8 -= speed_servo_arm_wristpitch;
    
    if (pos_servo_arm_wristPitch != aux_uint8){
        pos_servo_arm_wristPitch = aux_uint8;
        aux_msg_uint8.data = pos_servo_arm_wristPitch;
        servo_arm_wristPitch.publish(aux_msg_uint8);
    }

    // B and A buttons for gripper closure control ->
    aux_uint8 = pos_servo_arm_gripper;
    if ((msg -> BUTTON_B) && (pos_servo_arm_gripper < limit_servo_arm_gripper[1]))
        aux_uint8 += speed_servo_arm_gripper;
    if ((msg -> BUTTON_A) && (pos_servo_arm_gripper > limit_servo_arm_gripper[0]))
        aux_uint8 -= speed_servo_arm_gripper;
    
    if (pos_servo_arm_gripper != aux_uint8){
        pos_servo_arm_gripper = aux_uint8;
        aux_msg_uint8.data = aux_uint8;
        servo_arm_gripper.publish(aux_msg_uint8);
    }

    // A and B buttons for microscope control ->
    aux_uint8 = pos_servo_science_microscope;
    if ((msg -> BUTTON_A) && (pos_servo_science_microscope < limit_servo_science_microscope[1]))
        aux_uint8 += speed_servo_science_microscope;
    if ((msg -> BUTTON_B) && (pos_servo_science_microscope > limit_servo_science_microscope[0]))
        aux_uint8 -= speed_servo_science_microscope;
    
    if (pos_servo_science_microscope != aux_uint8){
        pos_servo_science_microscope = aux_uint8;
        aux_msg_uint8.data = pos_servo_science_microscope;
        servo_science_microscope.publish(aux_msg_uint8);
    }


    // Back and Play buttons for exterior dispencer ->
    aux_uint8 = pos_servo_science_dispenser_exterior;
    if ((msg -> BUTTON_BACK) && (pos_servo_science_dispenser_exterior < limit_servo_science_dispenser_exterior[1]))
        aux_uint8 += speed_servo_science_dispenser_exterior;
    if ((msg -> BUTTON_PLAY) && (pos_servo_science_dispenser_exterior > limit_servo_science_dispenser_exterior[0]))
        aux_uint8 -= speed_servo_science_dispenser_exterior;
    
    if (pos_servo_science_dispenser_exterior != aux_uint8){
        pos_servo_science_dispenser_exterior = aux_uint8;
        aux_msg_uint8.data = pos_servo_science_dispenser_exterior;
        servo_science_dispenser_exterior.publish(aux_msg_uint8);
    }


    // Left and Right Triggers for interior dispenser -> 
    aux_uint8 = pos_servo_science_dispenser_interior;
    if ((msg -> LEFT_TRIGGER) && (pos_servo_science_dispenser_interior < limit_servo_science_dispenser_exterior[1]))
        aux_uint8 += speed_servo_science_dispenser_interior;
    if ((msg -> BUTTON_LB) && (pos_servo_science_dispenser_interior) > limit_servo_science_dispenser_exterior[0])
        aux_uint8 -= speed_servo_science_dispenser_interior;
    
    if (pos_servo_science_dispenser_interior != aux_uint8){
        pos_servo_science_dispenser_interior = aux_uint8;
        aux_msg_uint8.data = pos_servo_science_dispenser_interior;
        servo_science_dispenser_interior.publish(aux_msg_uint8);
    }

}

#endif