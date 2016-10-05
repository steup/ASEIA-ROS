#include <ros/ros.h>
#include <ros/console.h>

#include <SensorEventPublisher.h>
#include <BaseEvent.h>
#include <ID.h>

struct WTConfig : public BaseConfig {
  //using PositionScale = std::ratio<1, 1000>;
	//using TimeScale = std::ratio<1000>;
  using TimeValueType = Value<float, 1>;
};

struct WSConfig : public BaseConfig {
  using PositionScale = std::ratio<1, 1000>;
	using TimeScale = std::ratio<1000>;
  using TimeValueType = Value<uint32_t, 1>;
};


struct EventPub {
  using WTE = BaseEvent<WTConfig>;
  SensorEventPublisher<WTE> tPub;
  WTE te;
  using WSE = BaseEvent<WSConfig>;
  SensorEventPublisher<WSE> sPub;
  WSE se;
  ros::Timer t;

  EventPub()
    : t(ros::NodeHandle().createTimer(ros::Duration(1.0), &EventPub::run, this))
  {
    te.attribute(id::attribute::Position())    = { { {1, 2} }, { {3,4} }, { {5, 6} } };
    te.attribute(id::attribute::Time())        = { { {ros::Time::now().toSec(), 1} } };
    te.attribute(id::attribute::PublisherID()) = { { {tPub.nodeId()} } };
    se.attribute(id::attribute::Position())    = { { {1000,2000} }, { {3000,4000} }, { {5000, 6000} } };
    se.attribute(id::attribute::Time())        = { { {(uint32_t)ros::Time::now().toSec()/1000, 1} } };
    se.attribute(id::attribute::PublisherID()) = { { {sPub.nodeId()} } };
  }

  void run(const ros::TimerEvent& msg){
    te.attribute(id::attribute::Time())        = { { {ros::Time::now().toSec(), 1} } };
    se.attribute(id::attribute::Time())        = { { {(uint32_t)ros::Time::now().toSec()/1000, 1} } };
    tPub.publish(te);
    sPub.publish(se);
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
