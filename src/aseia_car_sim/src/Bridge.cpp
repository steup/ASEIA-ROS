#include <string>
#include <map>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <SensorEventSubscriber.h>
#include <BaseEvent.h>
#include <Attribute.h>

using namespace std;
using namespace id::attribute;
using namespace ros;

map<string, Publisher> pubs;

const uint32_t UTM = 1;

struct UTMBaseConfig : public BaseConfig {
  using TimeValueType = Value<double, 1>;
  using PositionValueType = Value<float, 3>;
  using PositionScale = Scale<std::ratio<1>, UTM>;
};

struct RoadBaseConfig : public BaseConfig {
  using TimeValueType = Value<double, 1>;
  using PublisherIDValueType = Value<uint32_t, 1, 1, false>;
  using PositionValueType = Value<float, 3, 1, true>;
};

using ObjAttr = Attribute<Object, Value<uint32_t, 1, 1, false>>;
using AngAttr = Attribute<Angle, Value<float, 3>>;
using PoseEvent = BaseEvent<UTMBaseConfig>::append<ObjAttr>::type::append<AngAttr>::type;
using RoadPoseEvent = BaseEvent<RoadBaseConfig>::append<ObjAttr>::type;

void handlePoseInput(const PoseEvent& e) {
  uint32_t car = e.attribute(Object()).value()(0.0);
  string topic = "/car"+to_string(car)+"/pose";
  auto it = pubs.find(topic);
  if(it == pubs.end()) {
    NodeHandle nh;
    it = pubs.emplace(topic, Publisher(nh.advertise<nav_msgs::Odometry>(topic, 1))).first;
  }
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = e.attribute(Position()).value()(0,0).value();
  msg.pose.pose.position.y = e.attribute(Position()).value()(1,0).value();
  msg.pose.pose.position.z = 0;
  msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0f);
  msg.header.frame_id = "map";
  msg.child_frame_id = "car"+to_string(car);
  it->second.publish(msg);
}

void handleRoadInput(const RoadPoseEvent& e) {
  uint32_t car = e.attribute(Object()).value()(0.0);
  string topic = "/car"+to_string(car)+"/road";
  auto it = pubs.find(topic);
  if(it == pubs.end()) {
    NodeHandle nh;
    it = pubs.emplace(topic, Publisher(nh.advertise<std_msgs::String>(topic, 1))).first;
  }
  std_msgs::String msg;
  ostringstream os;
  os << e;
  msg.data = os.str();
  it->second.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "aseia_bridge");
  SensorEventSubscriber<PoseEvent> poseSub(handlePoseInput);
  SensorEventSubscriber<RoadPoseEvent> roadSub(handleRoadInput);
  while(ros::ok()) ros::spin();
  return 0;
}
