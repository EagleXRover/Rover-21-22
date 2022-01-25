#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"


#include <iostream>
#include <sstream>
#include <string>

#pragma once
struct BaseMsg{
    uint8_t gripper_hand = 0x00;
    uint8_t wrist_roll = 0x00;
    uint8_t wrist_pitch = 0x00;
    uint8_t foreArm_pitch = 0x00;
    uint8_t upperArm_pitch = 0x00;
    uint8_t baseArm_yaw = 0x00;
    uint8_t speed_wheels = 0x00;

    BaseMsg(const sensor_msgs::Joy::ConstPtr &inputMsg);
    void output(std_msgs::UInt16 &arm, std_msgs::UInt8 &wheels);
};

BaseMsg::BaseMsg(const sensor_msgs::Joy::ConstPtr& inputMsg){
    // A and B for gripper hand opening control->
    if (inputMsg->buttons[0] || inputMsg->buttons[1]) gripper_hand = 0x02;
    gripper_hand += inputMsg->buttons[0];

    // Horizontal right joystick for wrist roll->
    if (inputMsg->axes[3] > 0.25 || inputMsg->axes[3] < -0.25) wrist_roll = 0x02;
    wrist_roll += (inputMsg->axes[3] < -0.25);

    // Vertical right joystick for wrist pitch->
    if (inputMsg->axes[4] > 0.25 || inputMsg->axes[4] < -0.25) wrist_pitch = 0x02;
    wrist_pitch += (inputMsg->axes[4] < -0.25);

    // LB and RB for forearm pitch control->
    if (inputMsg->buttons[4] || inputMsg->buttons[5]) foreArm_pitch = 0x02;
    foreArm_pitch += inputMsg->buttons[4];

    // D-Pad Up and Down for upper arm pitch control->
    if (inputMsg->axes[7] > 0.25 || inputMsg->axes[7] < -0.25) upperArm_pitch = 0x02;
    upperArm_pitch += (inputMsg->axes[7] > 0.25);

    // D-Pad Left and Right for base arm yaw control->
    if (inputMsg->axes[6] > 0.25 || inputMsg->axes[6] < -0.25) baseArm_yaw = 0x02;
    baseArm_yaw += (inputMsg->axes[6] > 0.25);

    // Left joystick for wheels movements->
    float x = -8 * inputMsg->axes[0];
    float y = 8 * inputMsg->axes[1];
    speed_wheels = uint8_t((x+y)*0.7071067811865) << 4;
    speed_wheels += uint8_t((y-x)*0.7071067811865);
}

void BaseMsg::output(std_msgs::UInt16 &arm, std_msgs::UInt8 &wheels){
    wheels.data = speed_wheels;
    arm.data = 0;
    arm.data ^= uint16_t(baseArm_yaw) << 2*5;
    arm.data ^= uint16_t(upperArm_pitch) << 2*4;
    arm.data ^= uint16_t(foreArm_pitch) << 2*3;
    arm.data ^= uint16_t(wrist_pitch) << 2*2;
    arm.data ^= uint16_t(wrist_roll) << 2*1;
    arm.data ^= uint16_t(gripper_hand) << 2*0;
}


#ifndef BASE_NODE_H
#define BASE_NODE_H

class BaseNode{
    protected:
        ros::NodeHandle* nh;
        ros::Subscriber* sub;
        ros::Publisher* pub;
        bool consoleOutput;
    public:
        BaseNode(ros::NodeHandle *nh);
        BaseNode(ros::NodeHandle *nh, const bool consoleOutput);
        void setSubscriber(const char subTopic[]);
        void setPublisher(const char pubTopic0[], const char pubTopic1[]);
        void callback(const sensor_msgs::Joy::ConstPtr& msg);
        void publish(std_msgs::UInt16 &arm, std_msgs::UInt8 &wheels);
};

BaseNode::BaseNode(ros::NodeHandle *nh):
    nh(nh), sub(nullptr), pub(nullptr), consoleOutput(true){
}

BaseNode::BaseNode(ros::NodeHandle *nh, const bool consoleOutput):
    nh(nh), sub(nullptr), pub(nullptr), consoleOutput(consoleOutput){
}

void BaseNode::setSubscriber(const char subTopic[]){
    if (sub != nullptr) sub->shutdown();
    sub = new ros::Subscriber();
    *sub = nh->subscribe(subTopic, 1000, &BaseNode::callback, this);
}

void BaseNode::setPublisher(const char pubTopic0[], const char pubTopic1[]){
    if (pub != nullptr) {
        pub[0].shutdown();
        pub[1].shutdown();
        delete[] pub;
    }
    pub = new ros::Publisher[2];
    pub[0] = nh->advertise<std_msgs::UInt16>(pubTopic0, 10);
    pub[1] = nh->advertise<std_msgs::UInt8>(pubTopic1, 10);
}

void BaseNode::callback(const sensor_msgs::Joy::ConstPtr& msg){
    BaseMsg bMsg(msg);
    std_msgs::UInt16 *Node0 = new std_msgs::UInt16();
    std_msgs::UInt8 *Node1 = new std_msgs::UInt8();
    bMsg.output(*Node0, *Node1);
    if (consoleOutput){
        std::stringstream ss;
        ss << "I heard a joy... ";
        ROS_INFO(ss.str().c_str());
    }
    publish(*Node0, *Node1);
}
void BaseNode::publish(std_msgs::UInt16 &arm, std_msgs::UInt8 &wheels){
    if (consoleOutput){
        std::stringstream ss;
        ss << "I echoed: [" << int(arm.data) << ", " << int(wheels.data) << "]"; 
        ROS_INFO(ss.str().c_str());
    }
    pub[0].publish(arm);
    pub[1].publish(wheels);
}



#endif