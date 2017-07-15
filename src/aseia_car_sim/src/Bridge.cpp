#include "Attributes.h"
#include "Nurbs.h"

#include <string>
#include <map>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <SensorEventSubscriber.h>
#include <BaseEvent.h>
#include <Attribute.h>


namespace aseia_car_sim {

using namespace std;
using namespace id::attribute;
using namespace ros;
using namespace Eigen;
using id::attribute::Time;

map<string, Publisher> pubs;

struct EventConfig : public BaseConfig {
    using TimeValueType = Value<uint32_t, 1>;
    using TimeScale = Scale<std::ratio<1,1000>>;
    using PositionValueType = Value<float, 3>;
    using PositionScale = Scale<std::ratio<1>, 0>;
};

using UTMBaseConfig = EventConfig;

struct RoadBaseConfig : public EventConfig {
  using PositionScale = Scale<std::ratio<1>, 1>;
};

using ObjAttr = Attribute<Object, Value<uint32_t, 1, 1, false>>;
using OriAttr = Attribute<Orientation, Value<float, 4>, Radian>;
using DistAttr = Attribute<Distance, Value<float, 1>, Meter>;
using PoseEvent = BaseEvent<UTMBaseConfig>::append<ObjAttr>::type::append<OriAttr>::type;
using RoadPoseEvent = BaseEvent<RoadBaseConfig>::append<ObjAttr>::type::append<OriAttr>::type;
using UTMACCEvent = BaseEvent<UTMBaseConfig>::append<ObjAttr>::type::append<DistAttr>::type;
using RoadACCEvent = BaseEvent<RoadBaseConfig>::append<ObjAttr>::type::append<DistAttr>::type;

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
  msg.pose.pose.position.z = e.attribute(Position()).value()(2,0).value()+5.0;
  msg.pose.pose.orientation.x = e.attribute(Orientation()).value()(0,0).value(),
  msg.pose.pose.orientation.y = e.attribute(Orientation()).value()(1,0).value(),
  msg.pose.pose.orientation.z = e.attribute(Orientation()).value()(2,0).value(),
  msg.pose.pose.orientation.w = e.attribute(Orientation()).value()(3,0).value(),
  msg.header.frame_id = "map";
  msg.header.stamp.sec = e.attribute(Time()).value()(0,0).value()/1000;
  msg.header.stamp.nsec = (e.attribute(Time()).value()(0,0).value()%1000)*1000000;
  msg.child_frame_id = "car"+to_string(car);
  it->second.publish(msg);
}

void handleRoadInput(const RoadPoseEvent& e) {
  ROS_DEBUG_STREAM("Got Road Position Event: " << e);
  uint32_t car = e.attribute(Object()).value()(0,0);
  string topic = "/car"+to_string(car)+"/roadOffset";
  auto it = pubs.find(topic);
  if(it == pubs.end()) {
    NodeHandle nh;
    it = pubs.emplace(topic, Publisher(nh.advertise<std_msgs::Float32>(topic, 1))).first;
  }
  std_msgs::Float32 msg;
  msg.data = e.attribute(Position()).value()(2,0).value();
  it->second.publish(msg);
  topic = "/car"+to_string(car)+"/roadLane";
  it = pubs.find(topic);
  if(it == pubs.end()) {
    NodeHandle nh;
    it = pubs.emplace(topic, Publisher(nh.advertise<std_msgs::Float32>(topic, 1))).first;
  }
  msg.data = e.attribute(Position()).value()(1,0).value();
  it->second.publish(msg);
}

struct NurbsBaseConfig : public BaseConfig {
  using TimeValueType = Value<double, 1>;
  using PositionValueType = Value<double, 3>;
  using PositionScale = Scale<std::ratio<1>, 1>;
};
using Ref = Attribute<Reference, Value<double, 3>, Meter>;
using Ori = Attribute<Orientation, Value<double, 4>, Radian>;
using NurbData = Attribute<Nurbs, Value<double, 100, 4, false>, Meter, Scale<std::ratio<1>, 1>>;
using RoadEvent = BaseEvent<NurbsBaseConfig>
                        ::append<Ref>::type
                        ::append<NurbData>::type
                        ::append<Ori>::type;

template<typename T>
static ostream& operator<<(ostream& o, const vector<T>& v) {
  o << "[";
  for(const auto& elem : v)
    o << elem << " ";
  return o << "]";
}

static ostream& operator<<(ostream& o, const geometry_msgs::Point& p) {
  return o << "(" << p.x << " " << p.y << " " << p.z << ")" << endl;
}


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
  marker.pose.orientation.x = e.attribute(Orientation()).value()(0,0).value();
  marker.pose.orientation.y = e.attribute(Orientation()).value()(1,0).value();
  marker.pose.orientation.z = e.attribute(Orientation()).value()(2,0).value();
  marker.pose.orientation.w = e.attribute(Orientation()).value()(3,0).value();
  marker.lifetime = ros::Duration(60);
  marker.color.a = 1.0;
  marker.color.b = 1.0;
  marker.color.g = 1.0;
  marker.color.r = 1.0;
  // todo set line width
  //todo add points from nurb
  const auto& nurbData = e.attribute(Nurbs()).value();
  const size_t dim = (size_t)e.attribute(Nurbs()).value()(0,0);
  const size_t pSize = (size_t)e.attribute(Nurbs()).value()(0,1);
  const size_t lSize = (size_t)e.attribute(Nurbs()).value()(0,2);
  const auto& knots = nurbData.block(1, 3, lSize+1, 1);
  const auto& points = nurbData.block(1, 0, pSize+1, 3);
  marker.scale.x = 5.0;
  ROS_DEBUG_STREAM("NURBS:\tdim  : " << dim << endl <<
                         "\tpSize: " << pSize << endl <<
                         "\tlSize: " << lSize << endl <<
                         "\tknots: " << endl << knots << endl <<
                         "\tpoints: " << endl << points);
  NURBCurve c(dim, NURBCurve::KnotsType(knots), NURBCurve::PointsType(points));
  for(size_t i=300/lSize; i<100-300/lSize; i++) {
      NURBCurve::Point p = c.sample(i/100.0);
      geometry_msgs::Point temp;
      temp.x=p(0,0);
      temp.y=p(0,1);
      temp.z=p(0,2);
      marker.points.emplace_back(move(temp));
  }
  ROS_DEBUG_STREAM("NURBS: Result: " << endl << marker.points);
  it->second.publish(marker);
}

void handleRoadACC(const RoadACCEvent& e) {
  uint32_t car = e.attribute(Object()).value()(0.0);
  string topic = "/car"+to_string(car)+"/accRoad";
  auto it = pubs.find(topic);
  if(it == pubs.end()) {
    NodeHandle nh;
    it = pubs.emplace(topic, Publisher(nh.advertise<std_msgs::Float32>(topic, 1))).first;
  }
  ROS_DEBUG_STREAM("Road ACC Message: " << e);
  std_msgs::Float32 msg;
  msg.data = e.attribute(Distance()).value()(0,0).value();
  it->second.publish(msg);
}

void handleUTMACC(const UTMACCEvent& e) {
  uint32_t car = e.attribute(Object()).value()(0.0);
  string topic = "/car"+to_string(car)+"/accUTM";
  auto it = pubs.find(topic);
  if(it == pubs.end()) {
    NodeHandle nh;
    it = pubs.emplace(topic, Publisher(nh.advertise<std_msgs::Float32>(topic, 1))).first;
  }
  std_msgs::Float32 msg;
  msg.data = e.attribute(Distance()).value()(0,0);
  it->second.publish(msg);
}
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "aseia_bridge");
  SensorEventSubscriber<aseia_car_sim::PoseEvent> poseSub(aseia_car_sim::handlePoseInput);
  SensorEventSubscriber<aseia_car_sim::RoadPoseEvent> roadPoseSub(aseia_car_sim::handleRoadInput);
  SensorEventSubscriber<aseia_car_sim::RoadEvent> roadSub(aseia_car_sim::handleRoad);
  SensorEventSubscriber<aseia_car_sim::RoadACCEvent> roadAccSub(aseia_car_sim::handleRoadACC);
  SensorEventSubscriber<aseia_car_sim::UTMACCEvent> utmAccSub(aseia_car_sim::handleUTMACC);
  while(ros::ok()) ros::spin();
  return 0;
}
