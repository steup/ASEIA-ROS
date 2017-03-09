#include <string>
#include <map>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#include <SensorEventSubscriber.h>
#include <BaseEvent.h>
#include <Attribute.h>

using namespace std;
using namespace id::attribute;
using namespace ros;

map<string, Publisher> pubs;

const uint32_t UTM = 1;

struct EventConfig : public BaseConfig {
  using PositionScaleType = Scale<std::ratio<1>, UTM>;
};

using ObjAttr = Attribute<Object, Value<uint32_t, 1, 1, false>>;
using AngAttr = Attribute<Angle, Value<float, 1>>;
using GPSPoseEvent = BaseEvent<EventConfig>::append<ObjAttr>::type::append<AngAttr>::type;

void handleSensorInput(const GPSPoseEvent& e) {
  uint32_t car = e.attribute(Object()).value()(0.0);
  string topic = "/car"+to_string(car)+"/gps";
  auto it = pubs.find(topic);
  if(it == pubs.end()) {
    NodeHandle nh;
    pubs.emplace(topic, Publisher(nh.advertise<nav_msgs::Odometry>(topic, 1)));
  }
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = e.attribute(Position()).value()(0,0).value();
  msg.pose.pose.position.y = e.attribute(Position()).value()(1,0).value();
  msg.pose.pose.position.z = e.attribute(Position()).value()(2,0).value();
  msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(e.attribute(Angle()).value()(0,0).value());
  msg.header.frame_id = "map";
  msg.child_frame_id = "car"+car;
  it->second.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "aseia_bridge");
  SensorEventSubscriber<GPSPoseEvent> sub(handleSensorInput);
  while(ros::ok()) ros::spin();
  return 0;
}
