#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <ros/ros.h>
#include <sstream>

template <typename T>
class Publisher {
    protected:
        ros::NodeHandle* nh;
        ros::Publisher* pub;
        bool consoleOutput = true;
    public:
        Publisher(ros::NodeHandle *nh_);
        Publisher(ros::NodeHandle *nh_, const bool consoleOutput_);
        ~Publisher();
        void setPublisher(const char pubTopic[]);
        void publish(T& msg);
};

template <typename T>
Publisher<T>::Publisher(ros::NodeHandle *nh_):
    nh(nh_){
    pub = NULL;
}

template <typename T>
Publisher<T>::Publisher(ros::NodeHandle *nh_, const bool consoleOutput_):
    nh(nh_), consoleOutput(consoleOutput_){
    pub = NULL;
}

template <typename T>
Publisher<T>::~Publisher(){
    if (pub) pub->shutdown();
    nh = NULL;
    delete pub;
    pub = NULL;
}

template <typename T>
void Publisher<T>::setPublisher(const char pubTopic[]){
    if (pub) pub->shutdown();
    pub = new ros::Publisher();
    *pub = nh->advertise<T>(pubTopic, 10);
}

template <typename T>
void Publisher<T>::publish(T& msg){
    if (consoleOutput){
        std::stringstream ss;
        ss << "I published: [" << msg.data << "]"; 
        ROS_INFO(msg.data);
    }
    pub->publish(msg);
}

#endif