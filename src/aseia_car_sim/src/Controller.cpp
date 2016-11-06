#include "Controller.h"
#include "Data.h"
#include "Car.h"

#include <ros/ros.h>

namespace car {

  using namespace std;

  class PID : public Controller {
    public:
      PID(const string& path, Car& car)
        : Controller("pid", car, 0)
      {}

      std::string dataType() const { return "float"; }

      virtual bool operator()() {
        const Data* inPtr  = mCar.getSensor("lane");
        const Data* refPtr = mCar.getReference("laneOffset");
              Data* cmdPtr = mCar.getActuator("command");
        if( !inPtr || !refPtr || !cmdPtr ) {
          ROS_ERROR_STREAM("Lane controller got not enough data");
          return false;
        }
        const Float&      in  = dynamic_cast<const Float&  >(*inPtr);
        const Float&      ref = dynamic_cast<const Float&  >(*refPtr);
              FloatArray& cmd = dynamic_cast<FloatArray&>(*cmdPtr);
        cmd(0) = ( in.value() - ref.value() ) * 4;
        ROS_DEBUG_STREAM(*this);
        return true;
      }

      virtual void print(ostream& o) const {
        o << type() << ": (lanePosition - lanePositionRef) -> command";
      }
  };

  class Set : public Controller {
    public:
      Set(const string& path, Car& car)
        : Controller("cruise", car, 1)
      {}

      virtual bool operator()() {
        const Data* refPtr = mCar.getReference("speed");
              Data* cmdPtr = mCar.getActuator("command");
        if( !refPtr || !cmdPtr ) {
          ROS_ERROR_STREAM("Speed controller got not enough data");
          return false;
        }
        const Float&      ref = dynamic_cast<const Float&>(*refPtr);
              FloatArray& cmd = dynamic_cast<FloatArray&>(*cmdPtr);
        cmd(1) =  ref.value();
        ROS_DEBUG_STREAM(*this);
        return true;
      }

      virtual void print(ostream& o) const {
        o << type() << ": (speedRef) -> command";
      }
  };

  ControllerPtr createController(const string& path, Car& car) {
    string type;
    if(ros::param::get(path+"/type", type)) {
        if(type == "pid")
          return ControllerPtr(new PID(path, car));
        if(type == "set")
          return ControllerPtr(new Set(path, car));
    }
    return ControllerPtr();
  }
}
