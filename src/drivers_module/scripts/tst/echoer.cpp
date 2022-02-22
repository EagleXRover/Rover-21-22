#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>

const char node[] = "echoer_node";
const char sub_topic[] = "chatter";
const char pub_topic[] = "chatter_echo";


ros::Publisher pub;
void Callback(const std_msgs::String::ConstPtr& msg)
{
  pub.publish(msg);
  ROS_INFO("I echoed: [%s]", msg->data.c_str());
}
int main (int argc, char **argv)
{
    ros::init(argc, argv, node);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(sub_topic, 1000, Callback);
    pub = nh.advertise<std_msgs::String>(pub_topic, 10);
    ros::spin();
}