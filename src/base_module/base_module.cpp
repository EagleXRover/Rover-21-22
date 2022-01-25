#include "ros/ros.h"

#include "baseNode.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "Base");
  ros::NodeHandle n;
  BaseNode base_node(&n, true);
  base_node.setSubscriber("/joy");
  base_node.setPublisher("/arm", "/wheels");
  ros::spin();

  return 0;
}
