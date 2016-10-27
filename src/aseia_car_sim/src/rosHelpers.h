#pragma once

#include <ros/ros.h>

void exitRos(const std::string& msg) {
  ROS_FATAL(msg);
  ros::spinOnce();
  ros::shutdown();
}
