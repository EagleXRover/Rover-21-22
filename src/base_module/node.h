#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <sstream>
#include <stdexcept>


template <typename SubType, template PubType>
class Node{
    protected:
        ros::NodeHandle* nh;
        ros::Subscriber* sub;
        ros::Publisher* pub;
        bool consoleOutput;
        
    public:
        Node()
        ~Node();
        void setNodeHandler(const ros::NodeHandle* nh_);
        virtual void setSubscriber(const char subTopic[]);
        void setPublisher(const char pubTopic[]);
        virtual callback(const typename SubType::ConstPtr& msg);
        void publish(PubType& msg);
};

template <typename SubType, template PubType>
Node<SubType, PubType>::Node():
    nh(NULL), sub(NULL), pub(NULL), consoleOutput(true){
}

template <typename SubType, template PubType>
Node<SubType, PubType>::~Node(){
    if (nh) nh = NULL;
    if (sub) {
        sub -> shutdown();
        delete sub;
        sub = NULL;
    }
    if (pub) {
        pub -> shutdown();
        delete pub;
        pub = NULL;
    }
}

template <typename SubType, template PubType>
void Node<SubType, PubType>::setNodeHandler(const ros::NodeHandle* nh_){
    nh = nh_;
}

template <typename SubType, template PubType>
void Node<SubType, PubType>::setNodeHandler(const ros::NodeHandle* nh_){
    nh = nh_;
}

template <typename SubType, template PubType>
void Node<SubType, PubType>::setPublisher(const char pubTopic[]){
    if (pub) pub->shutdown();
    pub = new ros::Publisher();
    *pub = nh->advertise<PubType>(pubTopic, 10);
}

template <typename SubType, template PubType>
void Node<SubType, PubType>::publish(PubType& msg){
    if (!pub) {
        printf("Publisher has not been defined yet, please define it...\n")
        exit(EXIT_FAILURE);
    }
    pub->publish(msg);

    if (!consoleOutput) return;
    
    std::stringstream ss;
    ss << "I published: [" << msg.data << "]"; 
    ROS_INFO(ss.str().c_str());
}
#endif