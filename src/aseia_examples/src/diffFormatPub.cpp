#include <ros/ros.h>
#include <ros/console.h>

#include <SensorEventPublisher.h>
#include <BaseEvent.h>
#include <ID.h>

struct EventConfig : public BaseConfig {
  using PositionScale = std::ratio<1, 1000>;
	using TimeScale = std::ratio<1000>;
};


struct EventPub {
  using ThisEvent=BaseEvent<EventConfig>;
  SensorEventPublisher<ThisEvent> pub;
  ThisEvent e;
  ros::Timer t;

  EventPub()
    : t(ros::NodeHandle().createTimer(ros::Duration(1.0), &EventPub::run, this))
  {
    e.attribute(id::attribute::Position())    = { { {0,0} }, { {0,0} }, { {0, 0} } };
    e.attribute(id::attribute::Time())        = { { {(int64_t)ros::Time::now().toSec()/1000, 0} } };
    e.attribute(id::attribute::PublisherID()) = { { {pub.nodeId()} } };
  }

  void run(const ros::TimerEvent& msg){
    pub.publish(e);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaDiffPub");
  EventPub pub;

  ROS_INFO_STREAM("started");

  while(ros::ok())
    ros::spin();

  return 0;
}
