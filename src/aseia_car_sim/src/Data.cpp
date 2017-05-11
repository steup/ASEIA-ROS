#include <SensorEventPublisher.h>
#include <BaseEvent.h>
#include <ID.h>
#include <Attribute.h>

#include "Data.h"
#include "Car.h"
#include "VRepWrapper.h"

#include <ros/ros.h>

namespace car {

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
      struct LaneBaseConfig : public BaseConfig {
        using TimeValueType = Value<double, 1>;
        using PositionValueType = Value<float, 3>;
      };
      using Object = Attribute<id::attribute::Object, Value<uint32_t, 1, 1, false>>;
      using LaneEvent = BaseEvent<LaneBaseConfig>::append<Object>::type;
      LaneEvent mEvent;
      SensorEventPublisher<LaneEvent> mPub;
      VisionDepthSensor mSensor;
      float mPos;
      float roadTreshold = 0.9;
    public:
      LaneSensor(const std::string& path, const Car& car)
        : Float(path, car, true),
          mSensor(getName(path+"/handle"), car.index())
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
        value = ( (float)( start + stop ) / mSensor.resolution[0] ) - 1;
        ROS_DEBUG_STREAM(*this);
        mEvent.attribute(id::attribute::Time()).value()(0,0) = { ros::Time::now().toSec(), 0 };
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
      struct PoseBaseConfig : public BaseConfig {
        using TimeValueType = Value<double, 1>;
        using PositionValueType = Value<float, 3>;
        using PositionScale = Scale<ratio<1>, 1>;
      };
      using Object = Attribute<id::attribute::Object, Value<uint32_t, 1, 1, false>>;
      using Angle  = Attribute<id::attribute::Angle, Value<float, 3>>;
      using PoseEvent = BaseEvent<PoseBaseConfig>::append<Object>::type::append<Angle>::type;
      PoseEvent mEvent;
      SensorEventPublisher<PoseEvent> mPub;
    public:
      PoseSensor(const std::string& path, const Car& car)
        : Data("Pose", car, true)
      {
        ROS_INFO_STREAM("Add pose sensor for car " << car.index());
        mEvent.attribute(id::attribute::PublisherID()).value()(0,0) = mPub.nodeId();
        mEvent.attribute(id::attribute::Object()).value()(0,0) = car.index();
        update();
      }

      virtual bool update() {
        //Implement pose query
        ROS_DEBUG_STREAM(*this);
        mEvent.attribute(id::attribute::Time()).value()(0,0) = { ros::Time::now().toSec(), 0 };
        mEvent.attribute(id::attribute::Position()).value() = {{{0, 0}}, {{0, 0}}, {{ 0, 0 }}};
        mPub.publish(mEvent);
        return true;
      }

      virtual void print(ostream& o) const {
        o << "Pose sensor of car " << mCar.index();
      }
  };

  class VisionDistanceSensor: public Float {
    private:
      VisionDepthSensor mSensor;
      const float mMaxDist;
    public:
      VisionDistanceSensor(const std::string& path, const Car& car)
        : Float(path, car, true),
          mSensor(getName(path+"/handle"), car.index()),
          mMaxDist(getFloatParam(path+"/farClip")-getFloatParam(path+"/nearClip"))
      {
          ROS_INFO_STREAM("Add vision depth sensor " << getName(path+"/handle") << " with handle " << mSensor.handle);
          update();
      }
        virtual bool update() {
          VisionDepthSensor::Distances scan = mSensor.distances();
          value = numeric_limits<float>::infinity();
          for(ssize_t i = 0; i< mSensor.resolution[0]; i++)
            for(ssize_t j = 0; j< mSensor.resolution[1]; j++)
              if(scan(i,j) < value)
                value = scan(i, j);
          value *= mMaxDist;
          ROS_DEBUG_STREAM(*this);
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
