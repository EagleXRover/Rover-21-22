#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "baseNode.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "Base");
  ros::NodeHandle n;
  BaseNode base_node(&n, true);
  base_node.setSubscriber("/joy");
  base_node.setPublisher("/drivers/arm", "/drivers/wheels");
  ros::Publisher arduinoWatchdog = n.advertise<std_msgs::Empty>("/arduinoWatchdog", 3, false);
  std_msgs::Empty arduinoWatchdogMsg;

  
  ros::Rate loop_rate(10);
  while(ros::ok()){
    arduinoWatchdog.publish(arduinoWatchdogMsg);
    ros::spinOnce();
    // loop_rate.sleep();
  }
  return 0;
}
