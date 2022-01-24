#ifndef SIBSCRIBER_H
#define SIBSCRIBER_H

#include <ros/ros.h>

template <typename T>
class Subscriber {
    protected:
        ros::NodeHandle* nh;
        ros::Subscriber* sub;
        bool consoleOutput = true;
    public:
        Subscriber(ros::NodeHandle *nh_);
        Subscriber(ros::NodeHandle *nh_, const bool consoleOutput_);
        ~Subscriber();
        void setSubscriber(const char subTopic[]);
        void callback(const typename T::ConstPtr& msg);
};

template <typename T>
Subscriber<T>::Subscriber(ros::NodeHandle *nh_): 
    nh(nh_){
    sub = NULL;
}

template <typename T>
Subscriber<T>::Subscriber(ros::NodeHandle *nh_, const bool consoleOutput_):
    nh(nh_), consoleOutput(consoleOutput_){
    sub = NULL;
}

template <typename T>
Subscriber<T>::~Subscriber(){
    if (sub) sub->shutdown();
    nh = NULL;
    delete sub;
    sub = NULL;
}

template <typename T>
void Subscriber<T>::setSubscriber(const char subTopic[]){
    if (sub) sub->shutdown();
    sub = new ros::Subscriber();
    *sub = nh->subscribe(subTopic, 1000, &Subscriber::callback, this);
}

template <typename T>
void Subscriber<T>::callback(const typename T::ConstPtr& msg){
    if (consoleOutput){
        ROS_INFO("I heard: [%s]", msg->data.c_str());
    }
}

#endif