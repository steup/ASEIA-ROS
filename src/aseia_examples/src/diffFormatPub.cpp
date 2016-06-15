#include <ros/ros.h>
#include <ros/console.h>

#include <SensorEventPublisher.h>
#include <BaseEvent.h>
#include <ID.h>

struct EventConfig : public BaseConfig {
  using PositionValueType = Value<double, 2>;
	using TimeValueType = Value<double, 1, 1, false>;
};


struct EventPub {
  using ThisEvent=BaseEvent<EventConfig>;
  SensorEventPublisher<ThisEvent> pub;
  ThisEvent e;
  ros::Timer t;

  EventPub()
    : t(ros::NodeHandle().createTimer(ros::Duration(1.0), &EventPub::run, this))
  {
    e.attribute(id::attribute::Position())    = { { {0,0} }, { {0,0} } };
    e.attribute(id::attribute::Time())        = { { {ros::Time::now().toSec()} } };
    e.attribute(id::attribute::PublisherID()) = { { {0} } };
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
