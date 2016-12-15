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

  class LaneSensor : public Float {
    private:
      VisionDepthSensor mSensor;
      float mPos;
      float roadTreshold = 0.9;
    public:
      LaneSensor(const std::string& path, const Car& car)
        : Float(path, car, true),
          mSensor(getName(path+"/handle"), car.index())
      {
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
        return true;
      }

      virtual void print(ostream& o) const {
        o << "Lane Position of sensor " << mSensor.handle << ": " << value;
      }
  };

  class VisionDistanceSensor: public Float {
    private:
      VisionDepthSensor mSensor;
    public:
      VisionDistanceSensor(const std::string& path, const Car& car)
        : Float(path, car, true),
          mSensor(getName(path+"/handle"), car.index())
      {
          update();
      }
        virtual bool update() {
          VisionDepthSensor::Distances scan = mSensor.distances();
          value = numeric_limits<float>::infinity();
          for(ssize_t i = 0; i< mSensor.resolution[0]; i++)
            for(ssize_t j = 0; j< mSensor.resolution[1]; j++)
              if(scan(i,j) < value)
                value = scan(i, j);
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
        if(type == "proxDistance")
          return DataPtr(new ProximityDistanceSensor(path ,car));
    }
    return DataPtr();
  }
}
