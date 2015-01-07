#include <ros/ros.h>
#include <ros/console.h>

#include <unistd.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaTestPub");
  ros::NodeHandle n;
  ROS_INFO_STREAM("Aseia test publisher started");
  ros::spin();
  return 0;
}
