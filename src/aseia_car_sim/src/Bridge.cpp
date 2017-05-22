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
using namespace Eigen;

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
  marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(e.attribute(Orientation()).value()(0,0).value(),
                                                                    e.attribute(Orientation()).value()(1,0).value(),
                                                                    e.attribute(Orientation()).value()(2,0).value());
  marker.color.a = 1.0;
  marker.color.b = 1.0;
  marker.color.g = 1.0;
  marker.color.r = 1.0;
  // todo set line width
  //todo add points from nurb
  using Vector3T = Value<float, 1, 3, false>;
  const auto& nurbData = e.attribute(Nurbs()).value();
  const size_t dim = (size_t)e.attribute(Nurbs()).value()(0,0);
  const size_t pSize = (size_t)e.attribute(Nurbs()).value()(0,1);
  const size_t lSize = (size_t)e.attribute(Nurbs()).value()(0,2);
  marker.scale.x = 5.0;
  auto limits = nurbData.col(dim).segment(1,lSize+1);
  ROS_DEBUG_STREAM("NURBS: dim: " << dim);
  ROS_DEBUG_STREAM("NURBS: pSize: " << pSize);
  ROS_DEBUG_STREAM("NURBS: lSize: " << lSize);
  ROS_DEBUG_STREAM("NURBS: limits: " << endl << limits);
  ROS_DEBUG_STREAM("NURBS: points: " << endl << nurbData);
  auto f=[&limits](size_t i, size_t n, float u) {
      return (u-limits(i))/(limits(i+n)-limits(i));
  };
  auto g=[&limits](size_t i, size_t n, float u) {
      return (limits(i+n)-u)/(limits(i+n)-limits(i));
  };
  std::function<float(size_t,size_t,float)> N=[&limits, &f, &g, &N](size_t i, size_t n, float u) -> float {
      if(n==0)
          return (u >= limits(i) && u < limits(i+1))?1.0f:0.0f;
      else
          return f(i,n, u)*N(i, n-1, u) + g(i+1,n, u)*N(i+1, n-1, u);
  };
  ostringstream os;
  for(size_t i=300/lSize; i<100-300/lSize; i++) {
      auto point = Vector3T::Zeros();
      os << setprecision(3);
      for(size_t j=0; j<pSize; j++) {
          float n = N(j,3,i/100.0f);
          auto v = Vector3T(nurbData.block(j+1, 0, 1, dim));
          if(n>0.001) {
            os << "n(" << (i/100.0) << ", " << j << "): "
               << n << " * " << v;
          }
          point+=n*v;
      }
      geometry_msgs::Point temp;
      temp.x=point(0,0);
      temp.y=point(0,1);
      temp.z=point(0,2);
      marker.points.push_back(temp);
  }
  ROS_DEBUG_STREAM("NURBS intermediate:" << endl << os.str());
  ROS_DEBUG_STREAM("NURBS: Result: " << endl << marker.points);
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
