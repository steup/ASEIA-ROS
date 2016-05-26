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

struct DoorGridEventConfig : public BaseConfig
{
  using PositionValueType    = Value<int16_t, 2>;
  using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
  using PositionScale        = std::ratio<1, 100>;
};

using DoorGridAttribute = Attribute<OccupancyGrid, Value<uint8_t, 24, 24, false>, Dimensionless>;
using DoorGridEvent = BaseEvent<DoorGridEventConfig>::append<DoorGridAttribute>::type;


void forward(const DoorGridEvent& e){
  nav_msgs::OccupancyGrid msg;

  for(unsigned int y=0; y < 24; y++){
    for(unsigned int x=0; x < 24; x++){
      msg.data.push_back(e.attribute(OccupancyGrid()).value()(x,y));
    }
  }

  msg.header.stamp           = ros::Time::now();
  msg.header.frame_id        = "door_map";

  //msg.info.map_load_time     =
  msg.info.resolution        = 0.05f;
  msg.info.width             = 24;
  msg.info.height            = 24;
  msg.info.origin.position.x = e.attribute(Position()).value()(0);
  msg.info.origin.position.y = e.attribute(Position()).value()(1);
  msg.info.origin.position.z = 0;
  //msg.info.origin.quaternion.x
  //msg.info.origin.quaternion.y
  //msg.info.origin.quaternion.z
  //msg.info.origin.quaternion.w

  pub.publish(msg);
}

void print(const DoorGridEvent& e){
  ROS_INFO_STREAM("door: " << e);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaTestSub");

  ROS_INFO_STREAM("started");

  SensorEventSubscriber<DoorGridEvent> sub("door", 1, forward);

  ros::NodeHandle pubNode;
  pub = pubNode.advertise<nav_msgs::OccupancyGrid>("/patch_map", 1000);


  while(ros::ok())
    ros::spin();

  return 0;
}
