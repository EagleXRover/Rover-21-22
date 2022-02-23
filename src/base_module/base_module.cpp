#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "base_node.h"

#define nodeName_Base "base_node"
#define topicName_Joy "/joy"
#define topicName_Watchdog "/watchdog_topic"

int main(int argc, char **argv){
    ros::init(argc, argv, nodeName_Base);
    ros::NodeHandle n;

    BaseNode base_node(&n, topicName_Joy);

    ros::Publisher watchdog = n.advertise<std_msgs::Empty>(topicName_Watchdog, topic_queue_size, true);
    std_msgs::Empty watchdogMsg;

    ros::Rate loop_rate(10);
    while (ros::ok()){
        watchdog.publish(watchdogMsg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
