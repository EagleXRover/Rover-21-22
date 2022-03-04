#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/NavSatFix.h>

#define nodeName                "delta_corrdinates_pub"

#define topic_watchdog          "/watchdog_topic"

#define topic_gps_now           "/gps/actual"
#define topic_compass           "/gps/compass"

#define topic_gps_goal          "/gps/goal"

#define topic_gps_delta_rot     "/gps/delta/rot"
#define topic_gps_delta_dist    "/gps/delta/distance"

#define topic_queue_size    1

#define PI M_PI
#define R (6371e3) // metres
#define d2r (M_PI / 180.0)

float current_lat = 0;
float current_lon = 0;
float current_rot = 0;
float delta_lat = 0;
float delta_lon = 0;
float delta_rot = 0;
float delta_distance = 0;
float goal_lat = 0;
float goal_lon = 0;
float goal_rot = 0;


// https://stackoverflow.com/questions/365826/calculate-distance-between-2-gps-coordinates
double haversine_km(double lat1, double long1, double lat2, double long2)
{
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = R * c;

    return d;
}



void gpsCb(const sensor_msgs::NavSatFix &msg){
    current_lat = msg.latitude * PI / 180;
    current_lon = msg.longitude * PI / 180;
}

void rotCb(const std_msgs::Float32 &msg){
    current_rot = msg.data;
}

void goalCb(const sensor_msgs::NavSatFix &msg){
    goal_lat = msg.latitude * PI / 180;
    goal_lon = msg.longitude * PI / 180;

    delta_lat = goal_lat - current_lat;
    delta_lon = goal_lon - current_lon;

    delta_distance = haversine_km(current_lat*180/PI, current_lon*180/PI, msg.latitude, msg.longitude);

    goal_rot = atan2(goal_lon, goal_lat);

    delta_rot = goal_rot - current_rot;

    if (delta_rot < -PI) delta_rot += (2 * PI);
    if (delta_rot > PI) delta_rot += (-2 * PI);
}


int main(int argc, char** argv){
    ros::init(argc, argv, nodeName);
    ros::NodeHandle n;

    ros::Subscriber sub_gpsNow = n.subscribe(topic_gps_now, topic_queue_size, gpsCb);
    ros::Subscriber sub_compass = n.subscribe(topic_compass, topic_queue_size, rotCb);
    ros::Subscriber sub_gpsGoal = n.subscribe(topic_gps_goal, topic_queue_size, goalCb);


    ros::Publisher pub_watchdog = n.advertise<std_msgs::Empty>(topic_watchdog, topic_queue_size, true);
    ros::Publisher pub_rotation = n.advertise<std_msgs::Empty>(topic_gps_delta_rot, topic_queue_size, true);
    ros::Publisher pub_distance = n.advertise<std_msgs::Empty>(topic_gps_delta_dist, topic_queue_size, true);
    std_msgs::Empty watchdogMsg;
    std_msgs::Float32 auxFloat32msg;

    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();

        pub_watchdog.publish(watchdogMsg);
        auxFloat32msg.data = delta_rot;
        pub_rotation.publish(auxFloat32msg);
        auxFloat32msg.data = delta_distance;
        pub_watchdog.publish(auxFloat32msg);

        loop_rate.sleep();
    }
}