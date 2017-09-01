#include <SensorEventPublisher.h>
#include <SensorEventSubscriber.h>
#include <BaseEvent.h>
#include <ID.h>
#include <Attribute.h>
#include <Filter.h>

#include "Data.h"
#include "Car.h"
#include "VRepWrapper.h"
#include "Errors.h"

#include <ros/ros.h>

#include <Eigen/Geometry>

namespace car {

  struct EventConfig : public BaseConfig {
      using TimeValueType = Value<uint32_t, 1>;
      using TimeScale = Scale<std::ratio<1,1000>>;
      using PositionValueType = Value<float, 3>;
      using PositionScale = Scale<std::ratio<1>, 0>;
  };

  using namespace std;
  using namespace Eigen;
  using namespace vrep;

  static string getName(const string& name) {
    string temp;
    if (ros::param::get(name, temp))
      return temp;
    else
      return "";
  }

  static float getFloatParam(const string& name) {
    float temp;
    if (ros::param::get(name, temp))
      return temp;
    else
      return numeric_limits<float>::signaling_NaN();
  }


  class LaneSensor : public Float {
    private:
      struct LaneBaseConfig : public EventConfig {
        using PositionScale = Scale<std::ratio<1>, 1>;
      };
      using Object = Attribute<id::attribute::Object, Value<uint32_t, 1, 1, false>>;
      using LaneEvent = BaseEvent<LaneBaseConfig>::append<Object>::type;
      LaneEvent mEvent;
      SensorEventPublisher<LaneEvent> mPub;
      VisionDepthSensor mSensor;
      float mPos;
      float roadTreshold = 0.9;
      float mValue = 0, mAlpha;
    public:
      LaneSensor(const std::string& path, const Car& car)
        : Float(path, car, true),
          mSensor(getName(path+"/handle"), car.index()),
          mAlpha(getFloatParam(path+"/alpha"))
      {
        ROS_INFO_STREAM("Add lane sensor " << getName(path+"/handle") << " with handle " << mSensor.handle);
        mEvent.attribute(id::attribute::PublisherID()).value()(0,0) = mPub.nodeId();
        mEvent.attribute(id::attribute::Object()).value()(0,0) = car.index();
        update();
      }

      virtual bool update() {
        VisionDepthSensor::Distances scan = mSensor.distances();
        size_t start=0, stop=0;
        bool left = false;
        for( ssize_t i = 0; i < mSensor.resolution[0]; i++ ) {
          if(scan(i, 0) < roadTreshold ) {
            stop = i;
            left = true;
          }
          if ( !left )
            start = i;
        }
        value = (1-mAlpha)*mValue + mAlpha*(( (float)( start + stop ) / mSensor.resolution[0] ) - 1);
        mValue= value;
        ROS_DEBUG_STREAM(*this);
        mEvent.attribute(id::attribute::Time()).value()(0,0) = { getTime(), 0 };
        mEvent.attribute(id::attribute::Position()).value()(1,0) = { value, 0 };
        mPub.publish(mEvent);
        return true;
      }

      virtual void print(ostream& o) const {
        o << "Lane Position of sensor " << mSensor.handle << ": " << value;
      }
  };

  class PoseSensor : public Data {
    private:
      using PoseBaseConfig = EventConfig;
      using ObjectID = Attribute<id::attribute::Object, Value<uint32_t, 1, 1, false>>;
      using Ori  = Attribute<id::attribute::Orientation, Value<float, 4>, Radian>;
      using PoseEvent = BaseEvent<PoseBaseConfig>::append<ObjectID>::type::append<Ori>::type;
      PoseEvent mEvent;
      SensorEventPublisher<PoseEvent> mPub;
      Object mCarBody;
      using PosAttr  = PoseEvent::findAttribute<::id::attribute::Position>::type;
      using TimeAttr = PoseEvent::findAttribute<::id::attribute::Time>::type;
      using OriAttr  = PoseEvent::findAttribute<::id::attribute::Orientation>::type;
      NormalError<PosAttr> posError;
      NormalError<TimeAttr> timeError;
      //NormalError<OriAttr> oriError;
    public:
      PoseSensor(const std::string& path, const Car& car)
        : Data("Pose", car, true),
          mCarBody("CarBody", car.index())
      {
        ROS_INFO_STREAM("Add pose sensor for car " << car.index());
        mEvent.attribute(id::attribute::PublisherID()).value()(0,0) = mPub.nodeId();
        mEvent.attribute(id::attribute::Object()).value()(0,0) = car.index();
        update();
      }

      virtual bool update() {
        //Implement pose query
        const Object::Position pos = mCarBody.position();
        const Object::Orientation mod = Eigen::AngleAxisf(     0, Eigen::Vector3f::UnitX()) *
                                        Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitY()) *
                                        Eigen::AngleAxisf(     0, Eigen::Vector3f::UnitZ());
        const Object::Orientation ori = mCarBody.orientation() * mod;
        ROS_DEBUG_STREAM(*this);
        TimeAttr& timeAttr = mEvent.attribute(id::attribute::Time());
        PosAttr&  posAttr  = mEvent.attribute(id::attribute::Position());
        OriAttr&  oriAttr  = mEvent.attribute(id::attribute::Orientation());
        timeAttr.value()(0,0) = { getTime(), 0 };
        posAttr.value() = {{{pos[0], 0.2}}, {{pos[1], 0.2}}, {{ pos[2], 2 }}};
        oriAttr.value() = {{{ori.x(), 0}}, {{ori.y(), 0}}, {{ori.z(), 0 }}, {{ori.w(), 0 }}};
        timeAttr += timeError();
        posAttr  += posError();
        //ori  += oriError;
        ROS_DEBUG_STREAM("Car" << mCar.index() << ":\n" << mEvent);
        mPub.publish(mEvent);
        return true;
      }

      virtual void print(ostream& o) const {
        o << "Pose sensor of car " << mCar.index();
      }
  };

  class VisionDistanceSensor: public Float {
    private:
      using DistBaseConfig = EventConfig;
      using ObjectID = Attribute<id::attribute::Object, Value<uint32_t, 1, 1, false>>;
      using Distance = Attribute<id::attribute::Distance, Value<float, 1, 1, true>, Meter>;
      using DistEvent = BaseEvent<DistBaseConfig>::append<ObjectID>::type::append<Distance>::type;
      DistEvent mEvent;
      SensorEventPublisher<DistEvent> mPub;
      using DistAttr  = DistEvent::findAttribute<::id::attribute::Distance>::type;
      using PosAttr   = DistEvent::findAttribute<::id::attribute::Position>::type;
      using TimeAttr  = DistEvent::findAttribute<::id::attribute::Time>::type;
      using ObjectComp = decltype(DistEvent::findAttribute<::id::attribute::Object>::type());
      using UTMACCUComp = decltype(DistEvent::findAttribute<::id::attribute::Distance>::type().uncertainty());
      NormalError<TimeAttr> timeError;
      NormalError<DistAttr> distError;
      VisionDepthSensor mSensor;
      const float mMaxDist;
      UTMACCUComp c = {0.5};
      ObjectComp  o;
      using FilterExpr = decltype(filter::uncertainty(filter::e0[::id::attribute::Distance()]) < c && filter::e0[id::attribute::Object()] == o);
      SensorEventSubscriber<DistEvent, FilterExpr> mSub;
      float recvDist;
      size_t recvTimeout=0;
    public:
      void handleEvent(const DistEvent& e) {
        recvDist = e[id::attribute::Distance()].value()(0,0).value();
        recvTimeout = 10;
      }
      VisionDistanceSensor(const std::string& path, const Car& car)
        : Float(path, car, true),
          mSensor(getName(path+"/handle"), car.index()),
          mMaxDist(getFloatParam(path+"/farClip")-getFloatParam(path+"/nearClip")),
          o(car.index()),
          mSub(&VisionDistanceSensor::handleEvent, this,
                     filter::uncertainty(filter::e0[::id::attribute::Distance()]) < c && filter::e0[id::attribute::Object()] == o)
      {
          ROS_INFO_STREAM("Add vision depth sensor " << getName(path+"/handle") << " with handle " << mSensor.handle);
          mEvent.attribute(id::attribute::PublisherID()).value()(0,0) = mPub.nodeId();
          mEvent.attribute(id::attribute::Object()).value()(0,0) = car.index();
          update();
      }
        virtual bool update() {
          TimeAttr& timeAttr = mEvent.attribute(id::attribute::Time());
          DistAttr& distAttr = mEvent.attribute(id::attribute::Distance());
          timeAttr.value()(0,0) = { getTime(), 0 };
          timeAttr += timeError();
          VisionDepthSensor::Distances scan = mSensor.distances();
          value = numeric_limits<float>::infinity();
          for(ssize_t i = 0; i< mSensor.resolution[0]; i++)
            for(ssize_t j = 0; j< mSensor.resolution[1]; j++)
              if(scan(i,j) < value)
                value = scan(i, j);
          value *= mMaxDist;
          if(value > mMaxDist*0.9)
            distAttr.value()(0,0) = { mMaxDist/2, mMaxDist/2};
          else
            distAttr.value()(0,0) = { value, 1};
          distAttr += distError();
          ROS_DEBUG_STREAM(*this);
          mPub.publish(mEvent);
          if(recvTimeout>0) {
            value=recvDist;
            recvTimeout--;
          }
          return value != numeric_limits<float>::infinity();
        }

        virtual void print(ostream& o) const {
          o << "Vision Distance Sensor value: " << mSensor.handle << ": " << value;
        }
  };

  class ProximityDistanceSensor: public Float {
    private:
      ProximitySensor mSensor;
    public:
      ProximityDistanceSensor(const std::string& path, const Car& car)
        : Float(path, car, true),
          mSensor(getName(path+"/handle"), car.index())
      {
          ROS_INFO_STREAM("Add proximity sensor " << getName(path+"/handle") << " with handle " << mSensor.handle);
          update();
      }
      virtual bool update() {
        return mSensor.distance();
        ROS_DEBUG_STREAM(*this);
        return true;
      }

      virtual void print(ostream& o) const {
        o << "Proximity Distance Sensor value: " << mSensor.handle << ": " << value;
      }
  };

  DataPtr createData(const string& path, Car& car) {
    string type;
    ROS_INFO_STREAM("Qurying sensor type: " << path+"/type");
    if(ros::param::get(path+"/type", type)) {
        ROS_INFO_STREAM("Got sensor type: " << type);
        if(type == "laneSensor")
          return DataPtr(new LaneSensor(path, car));
        if(type == "float")
          return DataPtr(new Float(path ,car));
        if(type == "visionDistance")
          return DataPtr(new VisionDistanceSensor(path ,car));
        if(type == "pose")
          return DataPtr(new PoseSensor(path ,car));
    }
    return DataPtr();
  }
}
