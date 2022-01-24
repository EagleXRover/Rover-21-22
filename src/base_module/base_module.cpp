#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Byte.h"
#include "echoer.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "echoer");
  ros::NodeHandle n;
  Echoer<std_msgs::String> echo_node(&n);
  echo_node.setSubscriber("chatter");
  echo_node.setPublisher("chatter_echo");
  ros::spin();

  return 0;
}
