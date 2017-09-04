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

#include <aseia_car_sim/EventData.h>


namespace aseia_car_sim {

using namespace std;
using namespace id::attribute;
using namespace ros;
using namespace Eigen;
using id::attribute::Time;

struct EventConfig : public BaseConfig {
    using TimeValueType = Value<uint32_t, 1>;
    using TimeScale = Scale<std::ratio<1,1000>>;
    using PositionValueType = Value<float, 3>;
    using PositionScale = Scale<std::ratio<1>, 0>;
};

using UTMBaseConfig = EventConfig;

struct RoadBaseConfig : public EventConfig {
  using PositionScale = Scale<std::ratio<1>, 1>;
    using TimeScale = Scale<std::ratio<1,1000>, 1>;
};

using ObjAttr = Attribute<Object, Value<uint32_t, 1, 1, false>>;
using OriAttr = Attribute<Orientation, Value<float, 4>, Radian>;
using RoadDistAttr = Attribute<Distance, Value<float, 1>, Meter, Scale<std::ratio<1>, 1>>;
using RoadSpeedAttr = Attribute<Speed, Value<float, 1>, decltype(Meter()/Second()), Scale<std::ratio<1>, 1>>;
using UTMDistAttr = Attribute<Distance, Value<float, 1>, Meter>;
using UTMSpeedAttr = Attribute<Speed, Value<float, 1>, decltype(Meter()/Second())>;
using UTMPoseEvent = BaseEvent<UTMBaseConfig>::append<ObjAttr>::type::append<OriAttr>::type;
using RoadPoseEvent = BaseEvent<RoadBaseConfig>::append<ObjAttr>::type::append<OriAttr>::type;
using UTMACCEvent = BaseEvent<UTMBaseConfig>::append<ObjAttr>::type::append<UTMDistAttr>::type;
using RoadACCEvent = BaseEvent<RoadBaseConfig>::append<ObjAttr>::type::append<RoadDistAttr>::type;
using UTMSpeedEvent = BaseEvent<UTMBaseConfig>::append<ObjAttr>::type::append<UTMSpeedAttr>::type;
using RoadSpeedEvent = BaseEvent<RoadBaseConfig>::append<ObjAttr>::type::append<RoadSpeedAttr>::type;

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

struct NoFilter{};

template<typename InEvent, typename OutEvent, typename Filter>
class Receiver {
  protected:
    const string mBaseTopic;
    SensorEventSubscriber<InEvent, Filter> mSub;
    map<string, Publisher> mPubs;
  public:
    virtual bool handleEvent(const InEvent& e, OutEvent& msg) = 0;
    void helper(const InEvent& e) {
      OutEvent msg;
      if(!handleEvent(e, msg))
        return;
      uint32_t sec = e[Time()].value()(0,0).value()/1000UL;
      uint32_t nsec = (e[Time()].value()(0,0).value()%1000UL)*1000000UL;
      msg.header.stamp = ros::Time(sec, nsec);
      uint32_t car = e[Object()].value()(0,0);
      const string topic = "/car"+to_string(car)+"/"+mBaseTopic;
      ROS_DEBUG_STREAM("Got " << topic << " event:\n" << e);
      auto it = mPubs.find(topic);
      if(it == mPubs.end()) {
        NodeHandle nh;
        it = mPubs.emplace(topic, Publisher(nh.advertise<OutEvent>(topic, 1))).first;
      }
      it->second.publish(msg);
    }
    Receiver(const string& topic, const Filter& filter)
      : mBaseTopic(topic), mSub(&Receiver::helper, this, filter, 10)
    {}
};

template<typename InEvent, typename OutEvent>
class Receiver<InEvent, OutEvent, NoFilter> {
  protected:
    const string mBaseTopic;
    SensorEventSubscriber<InEvent> mSub;
    map<string, Publisher> mPubs;
  public:
    virtual bool handleEvent(const InEvent& e, OutEvent& msg) = 0;
    void helper(const InEvent& e) {
      OutEvent msg;
      if(!handleEvent(e, msg))
        return;
      uint32_t sec = e[Time()].value()(0,0).value()/1000UL;
      uint32_t nsec = (e[Time()].value()(0,0).value()%1000UL)*1000000UL;
      msg.header.stamp = ros::Time(sec, nsec);
      uint32_t car = e[Object()].value()(0,0);
      const string topic = "/car"+to_string(car)+"/"+mBaseTopic;
      ROS_DEBUG_STREAM("Got " << topic << " event:\n" << e);
      auto it = mPubs.find(topic);
      if(it == mPubs.end()) {
        NodeHandle nh;
        it = mPubs.emplace(topic, Publisher(nh.advertise<OutEvent>(topic, 1))).first;
      }
      it->second.publish(msg);
    }
    Receiver(const string& topic, const NoFilter& filter)
      : mBaseTopic(topic), mSub(&Receiver::helper, this, 10)
    {}
};


template<typename EventType, typename AttrID, typename Filter = NoFilter>
class Translator : public Receiver<EventType, EventData, Filter> {
  public:
    Translator(const string& topic, const Filter& filter = Filter())
      : Receiver<EventType, EventData, Filter>(topic, filter)
    {}
  virtual bool handleEvent(const EventType& e, EventData& msg) {
    const auto& v = e[AttrID()].value();
    if(v.rows()==0 || v.cols()==0)
      return false;
    for(size_t i = 0; i < v.rows(); i++)
      for(size_t j = 0; j < v.cols(); j++) {
      const auto& elem = v(i,j);
      msg.avg.push_back(elem.value());
      msg.min.push_back(elem.value()-elem.uncertainty());
      msg.max.push_back(elem.value()+elem.uncertainty());
    }
    return true;
  }
};

template<typename EventType>
class Odom : public Receiver<EventType, nav_msgs::Odometry, NoFilter> {
  public:
    Odom() : Receiver<EventType, nav_msgs::Odometry, NoFilter>("odom", NoFilter()) {}

    virtual bool handleEvent(const EventType& e, nav_msgs::Odometry& msg) {
      msg.pose.pose.position.x = e[Position()].value()(0,0).value();
      msg.pose.pose.position.y = e[Position()].value()(1,0).value();
      msg.pose.pose.position.z = e[Position()].value()(2,0).value()+5.0;
      msg.pose.pose.orientation.x = e[Orientation()].value()(0,0).value(),
      msg.pose.pose.orientation.y = e[Orientation()].value()(1,0).value(),
      msg.pose.pose.orientation.z = e[Orientation()].value()(2,0).value(),
      msg.pose.pose.orientation.w = e[Orientation()].value()(3,0).value(),
      msg.header.frame_id = "map";
      msg.child_frame_id = "car"+to_string(e[Object()].value()(0));
      return true;
    }
};


class RoadMarker {
  private:
    SensorEventSubscriber<RoadEvent> mRoadSub;
    Publisher mRoadPub;

  public:
    void handleRoad(const RoadEvent& e) {
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
      //marker.lifetime = ros::Duration(60);
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
      marker.scale.x = 10.0;
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
      mRoadPub.publish(marker);
    }

    RoadMarker()
      : mRoadSub(&RoadMarker::handleRoad, this),
        mRoadPub(ros::NodeHandle().advertise<visualization_msgs::Marker>("road", 1, true))
    {}
};

}
int main(int argc, char** argv) {
  ros::init(argc, argv, "aseia_bridge");
  using RoadACCUComp = decltype(aseia_car_sim::RoadACCEvent::findAttribute<::id::attribute::Distance>::type().uncertainty());
  using ObjectComp = decltype(aseia_car_sim::RoadACCEvent::findAttribute<::id::attribute::Object>::type());
  using UTMACCUComp = decltype(aseia_car_sim::UTMACCEvent::findAttribute<::id::attribute::Distance>::type().uncertainty());
  RoadACCUComp c = {0.5};
  UTMACCUComp c2 = {0.5};
  ObjectComp o = {0};
  auto roadACCFilter = filter::uncertainty(filter::e0[::id::attribute::Distance()]) < c && filter::e0[id::attribute::Object()] == o;
  auto utmACCFilter = filter::uncertainty(filter::e0[::id::attribute::Distance()]) < c2 && filter::e0[id::attribute::Object()] == o;
  aseia_car_sim::Translator<aseia_car_sim::RoadPoseEvent , id::attribute::Position> roadPoseSub ("roadPose");
  aseia_car_sim::Translator<aseia_car_sim::RoadSpeedEvent, id::attribute::Speed   > roadSpeedSub("roadSpeed");
  aseia_car_sim::Translator<aseia_car_sim::RoadACCEvent  , id::attribute::Distance, decltype(roadACCFilter)> roadACCSub  ("roadACC", roadACCFilter);
  aseia_car_sim::Translator<aseia_car_sim::UTMPoseEvent  , id::attribute::Position> utmPoseSub  ("utmPose");
  aseia_car_sim::Translator<aseia_car_sim::UTMSpeedEvent , id::attribute::Speed   > utmSpeedSub ("utmSpeed");
  aseia_car_sim::Translator<aseia_car_sim::UTMACCEvent   , id::attribute::Distance, decltype(utmACCFilter)> utmACCSub   ("utmACC", utmACCFilter);
  aseia_car_sim::Odom<aseia_car_sim::UTMPoseEvent> odomSub;
  aseia_car_sim::RoadMarker roadSub;
  ros::spin();
  return 0;
}
