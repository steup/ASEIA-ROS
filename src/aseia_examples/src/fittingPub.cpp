#include <ros/ros.h>
#include <ros/console.h>

#include <SensorEventPublisher.h>
#include <BaseEvent.h>
#include <ID.h>

using ThisEvent=BaseEvent<>;

SensorEventPublisher<ThisEvent> pub;
ThisEvent e;

void run(const ros::TimerEvent& msg){
    pub.publish(e);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaTestPub");

  ROS_INFO_STREAM("started");

  pub = std::move(SensorEventPublisher<ThisEvent>(0));

  e.attribute(id::attribute::Position())    = { { {0,0} }, { {0,0} }, { {0,0} } };
  e.attribute(id::attribute::Time())        = { { {(int64_t)ros::Time::now().toSec(), 0} } };
  e.attribute(id::attribute::PublisherID()) = { { {0} } };

  ros::Timer t= ros::NodeHandle().createTimer(ros::Duration(1.0), &run);
  
  while(ros::ok())
    ros::spin();
    
  return 0;
}
