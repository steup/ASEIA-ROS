#include <ros/ros.h>
#include <ros/console.h>

#include <SensorEventSubscriber.h>
#include <BaseEvent.h>

using namespace id::attribute;

struct DoorGridEventConfig : public BaseConfig
{
  using PositionValueType    = Value<int16_t, 2>;
  using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
  using ValidityValueType    = Value<uint8_t, 1, 1, false>;
  using PositionScale        = std::ratio<1, 100>;
  using ValidityScale        = std::ratio<1, 100 >; 
};

using DoorGridAttribute = Attribute<OccupancyGrid, Value<uint8_t, 24, 24, true>, Dimensionless>;
using DoorGridEvent = BaseEvent<DoorGridEventConfig>::append<DoorGridAttribute>::type;

void print(const DoorGridEvent& e){
  ROS_INFO_STREAM("received: " << e);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaTestSub");

  ROS_INFO_STREAM("started");

  SensorEventSubscriber<DoorGridEvent> sub("angle", 1, print);

  while(ros::ok())
    ros::spin();
    
  return 0;
}
