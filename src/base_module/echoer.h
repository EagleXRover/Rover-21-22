#ifndef ECHOER_H
#define ECHOER_H

#include <ros/ros.h>
#include "publisher.h"
#include "subscriber.h"

#include <sstream>
#include <string>


template <typename T>
class Echoer{
    protected:
        ros::NodeHandle* nh;
        ros::Subscriber* sub;
        ros::Publisher* pub;
        bool consoleOutput = true;
    public:
        Echoer(ros::NodeHandle *nh_);
        Echoer(ros::NodeHandle *nh_, const bool consoleOutput_);
        ~Echoer();
        void setSubscriber(const char subTopic[]);
        void setPublisher(const char pubTopic[]);
        void callback(const typename T::ConstPtr& msg);
        void publish(const T& msg);
};

template <typename T>
Echoer<T>::Echoer(ros::NodeHandle *nh_): 
    nh(nh_){
    sub = NULL;
    pub = NULL;
}

template <typename T>
Echoer<T>::Echoer(ros::NodeHandle *nh_, const bool consoleOutput_):
    nh(nh_), consoleOutput(consoleOutput_){
    sub = NULL;
    pub = NULL;
}

template <typename T>
Echoer<T>::~Echoer(){
    if (sub) sub->shutdown();
    if (pub) pub->shutdown();
    nh = NULL;
    delete sub;
    sub = NULL;
    delete pub;
    pub = NULL;
}

template <typename T>
void Echoer<T>::setSubscriber(const char subTopic[]){
    if (sub) sub->shutdown();
    sub = new ros::Subscriber();
    *sub = nh->subscribe(subTopic, 1000, &Echoer::callback, this);
}

template <typename T>
void Echoer<T>::setPublisher(const char pubTopic[]){
    if (pub) pub->shutdown();
    pub = new ros::Publisher();
    *pub = nh->advertise<T>(pubTopic, 10);
}

template <typename T>
void Echoer<T>::callback(const typename T::ConstPtr& msg){
    if (consoleOutput){
        T msg_;
        // msg_.data = msg->data;
        std::stringstream ss;
        ss << "I heard: [" << msg->data.c_str() << "]";
        ROS_INFO(ss.str().c_str());
        publish(*msg);
    }
}

template <typename T>
void Echoer<T>::publish(const T& msg){
    if (consoleOutput){
        std::stringstream ss;
        ss << "I echoed: [" << msg.data.c_str() << "]"; 
        ROS_INFO(ss.str().c_str());
    }
    pub->publish(msg);
}


#endif