#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "base_node.h"

#define nodeName_Base "Base node"
#define topicName_Joy "/joy"
#define topicName_Watchdog "/watchdogTopic"

int main(int argc, char **argv){
    ros::init(argc, argv, nodeName_Base);
    ros::NodeHandle n;

    
    ros::Publisher Watchdog = n.advertise<std_msgs::Empty>(topicName_Watchdog, topic_queue_size, true);
    std_msgs::Empty WatchdogMsg;

    //ros::Rate loop_rate(10);
    while (ros::ok()){
        Watchdog.publish(WatchdogMsg);
        ros::spinOnce();
        // loop_rate.sleep()
    }
}
