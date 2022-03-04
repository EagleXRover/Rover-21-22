#include "ros/ros.h"
#include "std_msgs/Empty.h"

#define nodeName                "delta_corrdinates_pub"
#define topic_gps_now           "/gps/actual"
#define topic_compass           "/gps/compass"

#define topic_gps_goal          "/gps/goal"

#define topic_gps_delta_rot     "/gps/delta/rot"
#define topic_gps_delta_dist    "/gps/delta/distance"

void gpsCb(){
    
}

void rotCb(){

}

void pubDeltas(){

}

int main(int argc, char** argv){
    ros::init(arc, argv, nodeName_Base);
    ros::NodeHandle n;

    ros::Publisher watchdog = n.advertise<std_msgs::Empty>(topicName_Watchdog, topic_queue_size, true);
    std_msgs::Empty watchdogMsg;

    ros::Rate loop_rate(10);
    while (ros::ok()){
        watchdog.publish(watchdogMsg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}