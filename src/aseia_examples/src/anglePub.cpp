#include <ros/ros.h>
#include <ros/console.h>

#include <SensorEventPublisher.h>
#include <BaseEvent.h>
#include <ID.h>

using namespace id::attribute;

struct AngleEventConfig : public BaseConfig
{
  using PositionValueType    = Value<int16_t, 2>;
  using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
  using ValidityValueType    = Value<uint8_t, 1, 1, false>;
  using PositionScale        = std::ratio<1, 100>;
  using ValidityScale        = std::ratio<1, 100 >;
    
};

using AngleAttribute = Attribute<Angle, Value<int16_t, 1>, Radian,  std::ratio<1, 100>>;
using ThisEvent = BaseEvent<AngleEventConfig>::append<AngleAttribute>::type;

SensorEventPublisher<ThisEvent> pub;
ThisEvent e;

void run(const ros::TimerEvent& msg){
    pub.publish(e);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AnglePub");

  ROS_INFO_STREAM("started");

  pub = std::move(SensorEventPublisher<ThisEvent>("angle", 0));

  e.attribute(Position()).value()    = {{100, 20}, {200,20}};
  e.attribute(Time()).value()        = {{(unsigned long)std::time(nullptr), 1}};
  e.attribute(PublisherID()).value() = {{55}};
  e.attribute(Validity()).value()    = {{80}};
  e.attribute(Angle()).value()       = {{314}};

  ros::Timer t= ros::NodeHandle().createTimer(ros::Duration(1.0), &run);
  
  while(ros::ok())
    ros::spin();
    
  return 0;
}
