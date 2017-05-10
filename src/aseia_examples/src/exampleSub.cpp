#include <ros/ros.h>
#include <ros/console.h>

#include <SensorEventSubscriber.h>
#include <BaseEvent.h>

struct EventConfig : BaseConfig {
  using TimeValueType = Value<uint32_t, 1>;
};
using ThisEvent = BaseEvent<EventConfig>;

void print(const ThisEvent& e){
  ROS_INFO_STREAM("received: " << e);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaTestSub");

  ROS_INFO_STREAM("started");
  SensorEventSubscriber<ThisEvent> sub(print, 10);

  while(ros::ok())
    ros::spin();

  return 0;
}
