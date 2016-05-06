#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>

#include <SensorEventSubscriber.h>
#include <BaseEvent.h>

using namespace id::attribute;

ros::Publisher pub;

struct RobotGridEventConfig : public BaseConfig
{
  using PositionValueType    = Value<int16_t, 2>;
  using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
  using PositionScale        = std::ratio<1, 100>;
};

using RobotGridAttribute = Attribute<OccupancyGrid, Value<uint8_t, 14, 14, false>, Dimensionless>;
using RobotGridEvent = BaseEvent<RobotGridEventConfig>::append<RobotGridAttribute>::type;


void print(const RobotGridEvent& e){
  ROS_INFO_STREAM("robot: " << e);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaTestSub");

  ROS_INFO_STREAM("started");

  SensorEventSubscriber<RobotGridEvent> sub("robot", 1, print);


  while(ros::ok())
    ros::spin();

  return 0;
}
