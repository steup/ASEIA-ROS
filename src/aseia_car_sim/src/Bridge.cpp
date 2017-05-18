#include "Attributes.h"

#include <string>
#include <map>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <SensorEventSubscriber.h>
#include <BaseEvent.h>
#include <Attribute.h>

namespace aseia_car_sim {

using namespace std;
using namespace id::attribute;
using namespace ros;

map<string, Publisher> pubs;

struct UTMBaseConfig : public BaseConfig {
  using TimeValueType = Value<double, 1>;
  using PositionValueType = Value<float, 3>;
};

struct RoadBaseConfig : public BaseConfig {
  using TimeValueType = Value<double, 1>;
  using PositionValueType = Value<float, 3>;
  using PositionScale = Scale<std::ratio<1>, 1>;
};

using ObjAttr = Attribute<Object, Value<uint32_t, 1, 1, false>>;
using AngAttr = Attribute<Angle, Value<float, 3>>;
using PoseEvent = BaseEvent<UTMBaseConfig>::append<ObjAttr>::type::append<AngAttr>::type;
using RoadPoseEvent = BaseEvent<RoadBaseConfig>::append<ObjAttr>::type::append<AngAttr>::type;

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

struct NurbsBaseConfig : public BaseConfig {
  using TimeValueType = Value<double, 1>;
  using PositionValueType = Value<float, 3>;
  using PositionScale = Scale<std::ratio<1>, 1>;
};
using Ref = Attribute<Reference, Value<float, 3>, Meter>;
using Ori = Attribute<Orientation, Value<float, 3>, Radian>;
using NurbData = Attribute<Nurbs, Value<float, 100, 4, false>, Meter, Scale<std::ratio<1>, 1>>;
using RoadEvent = BaseEvent<NurbsBaseConfig>
                        ::append<Ref>::type
                        ::append<NurbData>::type
                        ::append<Ori>::type;

void handleRoad(const RoadEvent& e) {
  string topic = "/roads/1";
  auto it = pubs.find(topic);
  if(it == pubs.end()) {
    NodeHandle nh;
    it = pubs.emplace(topic, Publisher(nh.advertise<visualization_msgs::Marker>(topic, 1))).first;
  }
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(e.attribute(id::attribute::Time()).value()(0,0).value());
  marker.ns = "roads";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = e.attribute(Reference()).value()(0,0).value();
  marker.pose.position.y = e.attribute(Reference()).value()(1,0).value();
  marker.pose.position.z = e.attribute(Reference()).value()(2,0).value();
  marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(e.attribute(Orientation()).value()(0,0).value(),
                                                                    e.attribute(Orientation()).value()(1,0).value(),
                                                                    e.attribute(Orientation()).value()(2,0).value());
  marker.color.a = 1.0;
  marker.color.b = 1.0;
  marker.color.g = 1.0;
  marker.color.r = 1.0;
  // todo set line width
  marker.scale.x = 1.0;
  //todo add points from nurb

  auto f=[limits](size_t i, size_t n, float u) {
      return (u-limits(i))/(limits(i+n)-limits(i));
  }
  auto g=[limits](size_t i, size_t n, float u) {
      return (limits(i+n)-u)/(limits(i+n)-limits(i));
  }
  auto N=[limits](size_t i, size_t n, float u):
      if(n==0) {
          if(u >= limits(i) && u < limits(i+1)):
              return 1;
          else
              return 0;
      } else
          return f(i,n, u)*N(i, n-1, u) + g(i+1,n, u)*N(i+1, n-1, u);
  }
  p=[]
  for i in range(0, 100):
      auto point = np.zeros(3);
      u = float(i)/100
      size_t j=0;
      for(const auto& p : P)
          point=point+N(j,3,u)*P(j)
      marker.points.push_back(point);
  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = 0.0;
  marker.points.push_back(p);
  p.x = 1.0;
  p.y = 1.0;
  p.z = 1.0;
  marker.points.push_back(p);
  it->second.publish(marker);
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "aseia_bridge");
  SensorEventSubscriber<aseia_car_sim::PoseEvent> poseSub(aseia_car_sim::handlePoseInput);
  SensorEventSubscriber<aseia_car_sim::RoadPoseEvent> roadPoseSub(aseia_car_sim::handleRoadInput);
  SensorEventSubscriber<aseia_car_sim::RoadEvent> roadSub(aseia_car_sim::handleRoad);
  while(ros::ok()) ros::spin();
  return 0;
}
