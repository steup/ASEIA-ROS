#include <ros/ros.h>
#include <ros/console.h>

#include <SensorEventSubscriber.h>
#include <BaseEvent.h>

void print(const BaseEvent<>& e){
  ROS_INFO_STREAM("received: " << e);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaTestSub");

  ROS_INFO_STREAM("started");

  SensorEventSubscriber<BaseEvent<>> sub(1, print);

  while(ros::ok())
    ros::spin();
    
  return 0;
}
