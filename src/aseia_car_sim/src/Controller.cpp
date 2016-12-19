#include "Controller.h"
#include "Data.h"
#include "Car.h"

#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace car {

  using namespace std;

  static float getFloatParam(const std::string& name) {
    float temp;
    if(ros::param::get(name, temp))
      return temp;
    else
      return 0.0f;
  }

  static string getStringParam(const std::string& name) {
    string temp;
    if(ros::param::get(name, temp))
      return temp;
    else
      return "";
  }

  class PID : public Controller {
    private:
      const float mKp, mKi, mKd;
      const string mSensorName, mRefName, mActName;
      float mI = 0.0f, mE = 0.0f;
      ros::Publisher mSensPub, mActPub;
    public:
      PID(const string& path, Car& car)
        : Controller("pid", car, 1),
          mKp(getFloatParam(path+"/p")),
          mKi(getFloatParam(path+"/i")),
          mKd(getFloatParam(path+"/d")),
          mSensorName(getStringParam(path+"/sensor")),
          mRefName(getStringParam(path+"/ref")),
          mActName(getStringParam(path+"/act"))
      {
        mSensPub = ros::NodeHandle().advertise<std_msgs::Float32>(mSensorName, 1);
        mActPub = ros::NodeHandle().advertise<std_msgs::Float32>(mActName, 1);
      }

      std::string dataType() const { return "float"; }

      virtual bool operator()() {
        const Data* inPtr  = mCar.getSensor(mSensorName);
        const Data* refPtr = mCar.getReference(mRefName);
              Data* actPtr = mCar.getActuator(mActName);
        if( !inPtr || !refPtr || !actPtr ) {
          ROS_ERROR_STREAM("PID controller got not enough data");
          return false;
        }
        const Float& in  = dynamic_cast<const Float&  >(*inPtr);
        const Float& ref = dynamic_cast<const Float&  >(*refPtr);
              Float& act = dynamic_cast<Float&>(*actPtr);
        float newE = in.value - ref.value;
        float d = newE - mE;
        mE = newE;
        mI += newE;
        act.value +=  mE * mKp + d * mKd + mI * mKi;
        std_msgs::Float32 msg;
        msg.data = in.value;
        mSensPub.publish(msg);
        msg.data = act.value;
        mActPub.publish(msg);
        ROS_DEBUG_STREAM(*this << ": " << in.value << " - " << ref.value << " -> " << act.value);
        return true;
      }

      virtual void print(ostream& o) const {
        o << type() << ": " << mActName << " + PID[" << mKp << ", " << mKi << ", " << mKd << "]("
          << mSensorName << " - " << mRefName << ") -> " << mActName;
      }
  };

  class Assign : public Controller {
    private:
      const string mActName, mRefName;
    public:
      Assign(const string& path, Car& car)
        : Controller("assign", car, 0),
          mActName(getStringParam(path+"/act")),
          mRefName(getStringParam(path+"/ref"))
      {}

      virtual bool operator()() {
        const Data* refPtr = mCar.getReference(mRefName);
              Data* actPtr = mCar.getActuator(mActName);
        if( !refPtr || !actPtr ) {
          ROS_ERROR_STREAM("Assignment got not enough data");
          return false;
        }
        const Float& ref = dynamic_cast<const Float&>(*refPtr);
              Float& act = dynamic_cast<Float&>(*actPtr);
        act.value =  ref.value;
        ROS_DEBUG_STREAM(*this);
        return true;
      }

      virtual void print(ostream& o) const {
        o << type() << ": " << mRefName << " -> " << mActName;
      }
  };

  class Min : public Controller {
    private:
      const string mActName, mRefName;
    public:
      Min(const string& path, Car& car)
        : Controller("min", car, 2),
          mActName(getStringParam(path+"/act")),
          mRefName(getStringParam(path+"/ref"))
      {}

      virtual bool operator()() {
        const Data* refPtr = mCar.getReference(mRefName);
              Data* actPtr = mCar.getActuator(mActName);
        if( !refPtr || !actPtr ) {
          ROS_ERROR_STREAM("Min got not enough data");
          return false;
        }
        const Float& ref = dynamic_cast<const Float&>(*refPtr);
              Float& act = dynamic_cast<Float&>(*actPtr);
        act.value =  min(act.value, ref.value);
        ROS_DEBUG_STREAM(*this << ": min(" << ref.value << ", " << act.value << ")");
        return true;
      }

      virtual void print(ostream& o) const {
        o << type() << ": min(" << mRefName << ", " << mActName << ") -> " << mActName;
      }
  };

  class Max : public Controller {
    private:
      const string mActName, mRefName;
    public:
      Max(const string& path, Car& car)
        : Controller("max", car, 2),
          mActName(getStringParam(path+"/act")),
          mRefName(getStringParam(path+"/ref"))
      {}

      virtual bool operator()() {
        const Data* refPtr = mCar.getReference(mRefName);
              Data* actPtr = mCar.getActuator(mActName);
        if( !refPtr || !actPtr ) {
          ROS_ERROR_STREAM("Min got not enough data");
          return false;
        }
        const Float& ref = dynamic_cast<const Float&>(*refPtr);
              Float& act = dynamic_cast<Float&>(*actPtr);
        act.value =  max(act.value, ref.value);
        ROS_DEBUG_STREAM(*this << ": max(" << ref.value << ", " << act.value << ")");
        return true;
      }

      virtual void print(ostream& o) const {
        o << type() << ": max(" << mRefName << ", " << mActName << ") -> " << mActName;
      }
  };

  ControllerPtr createController(const string& path, Car& car) {
    string type;
    if(ros::param::get(path+"/type", type)) {
        if(type == "pid")
          return ControllerPtr(new PID(path, car));
        if(type == "assign")
          return ControllerPtr(new Assign(path, car));
        if(type == "min")
          return ControllerPtr(new Min(path, car));
        if(type == "max")
          return ControllerPtr(new Max(path, car));
    }
    return ControllerPtr();
  }
}
