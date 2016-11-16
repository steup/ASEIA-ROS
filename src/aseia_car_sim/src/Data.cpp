#include "Data.h"
#include "Car.h"

#include <ros/ros.h>

#include <vrep_common/simRosGetVisionSensorDepthBuffer.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosGetObjectPose.h>
#include <vrep_common/simRosSetJointTargetPosition.h>
#include <vrep_common/simRosSetJointTargetVelocity.h>

#include <Eigen/Core>
#include <cmath>

namespace car {

  using namespace std;
  using namespace vrep_common;

  using A2 = Eigen::Array2f;
  using V3 = Eigen::Vector3f;

  static ros::ServiceClient handleSrv, poseSrv, depthSrv, jointPosSrv, jointVelSrv;

  static int getHandle(const string& objectName, int index = -1) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosGetObjectHandle >( "/vrep/simRosGetObjectHandle", true);
    simRosGetObjectHandle handle;
    handle.request.objectName = objectName;
    if(index != -1)
      handle.request.objectName += "#"+to_string(index);
    if( !srv.call(handle) ) {
      ROS_ERROR_STREAM("Service call to get handle of "<<handle.request.objectName<<" failed");
      return -1;
    }
    if(handle.response.handle == -1)
      ROS_ERROR_STREAM("No V-REP object with name "<<handle.request.objectName<<" found");
    return handle.response.handle;
  }

  static V3 getPosition(int handle, int reference) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosGetObjectPose >( "/vrep/simRosGetObjectPose", true);
    simRosGetObjectPose pose;
    pose.request.handle = handle;
    pose.request.relativeToObjectHandle = reference;
    if( !srv.call(pose) || pose.response.result == -1 )
      ROS_ERROR_STREAM("Error getting pose of object with handle " << handle);
    float x,y,z;
    x = pose.response.pose.pose.position.x;
    y = pose.response.pose.pose.position.y;
    z = pose.response.pose.pose.position.z;
    return V3(x, y, z);
  }

  static bool setAngle(int handle, float angle) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosSetJointTargetPosition >( "/vrep/simRosSetJointTargetPosition", true);
    simRosSetJointTargetPosition data;
    data.request.handle = handle;
    data.request.targetPosition = angle;
    if( !srv.call(data) || data.response.result == -1 ) {
      ROS_ERROR_STREAM("Error setting angle of joint handle " << handle);
      return false;
    }
    return true;
  }

  static bool setSpeed(int handle, float vel) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosSetJointTargetVelocity >( "/vrep/simRosSetJointTargetVelocity", true);
    simRosSetJointTargetVelocity data;
    data.request.handle = handle;
    data.request.targetVelocity = vel;
    if( !srv.call(data) || data.response.result == -1 ) {
      ROS_ERROR_STREAM("Error setting speed of joint handle " << handle);
      return false;
    }
    return true;
  }

  class Command : public FloatArray {
    private:
      ros::NodeHandle mNh;
      ros::ServiceClient mPoseSrv, mVelSrv, mPosSrv;
      int mLeftSteer, mRightSteer, mLeftMotor, mRightMotor;
      float mWidth, mLength;
      using Base = FloatArray;

      bool setAngles(A2 data) {
        return setAngle(mLeftSteer, data(0)) && setAngle(mRightSteer, data(1));
      }

      bool setSpeeds(A2 data) {
        return setSpeed(mLeftMotor, data(0)) && setSpeed(mRightMotor, data(1));
      }

    public:
      static const size_t steerIndex = 0;
      static const size_t speedIndex = 1;
      Command(const std::string& path, const Car& car)
        : Base(path, car, 2, false, true)
      {
          string name;
          ros::param::get(path+"/lSteer", name);
          mLeftSteer = getHandle(name, car.index());
          ros::param::get(path+"/rSteer", name);
          mRightSteer = getHandle(name, car.index());
          ros::param::get(path+"/lMotor", name);
          mLeftMotor = getHandle(name, car.index());
          ros::param::get(path+"/rMotor", name);
          mRightMotor = getHandle(name, car.index());
          ros::param::get(path+"/car", name);
          int carHandle = getHandle(name, car.index());
          ros::param::get(path+"/blAxis", name);
          int bl = getHandle(name, car.index());
          mWidth = (getPosition(mLeftSteer, carHandle)-getPosition(mRightMotor, carHandle)).norm()/2;
          mLength = (getPosition(mLeftSteer, carHandle)-getPosition(bl, carHandle)).norm();
      }
      float steer() const { return this->operator()(steerIndex); }
      void steer(float value) { this->operator()(steerIndex) = value; }
      float speed() const { return this->operator()(speedIndex); }
      void speed(float value) { this->operator()(speedIndex) = value; }
      void print(ostream& o) const { o << "Command: "; this->Base::print(o); }
      bool update() {
        //compute radius of curve
        A2 speeds = A2::Constant(speed());
        A2 angles = A2::Zero();
        if(steer() > 0.0001 || steer() < -0.0001) {
          float r = mLength * tanf(M_PI_2 - steer());
          float a = mWidth / ( r * r + mLength * mLength );
          A2 s(copysign(steer(), 1.0f), -copysign(steer(), 1.0f));
          speeds  *= (s * r * a + mWidth * a + 1).sqrt();

          //compute left and right angle of wheels dependent on radius of curve
          angles = ( mLength / ( s * mWidth + r) ).unaryExpr(ptr_fun(atanf));
        }
        ROS_DEBUG_STREAM("Speed: " << speeds);
        ROS_DEBUG_STREAM("Angle: " << angles);
        if(!setSpeeds(speeds) || !setAngles(angles))
        ROS_DEBUG_STREAM(*this);
        return true;
      }
  };


  class LaneSensor : public Float {
    private:
      simRosGetVisionSensorDepthBuffer mScan;
      float mPos;
      ros::ServiceClient srv;
      float roadTreshold = 0.9;
    public:
      using Resolution = pair<unsigned int, unsigned int>;
      LaneSensor(const std::string& path, const Car& car)
        : Float(path, car, true)
      {
        string name;
        if( !ros::param::get(path+"/handle", name) )
          ROS_ERROR_STREAM("No V-REP sensor name supplied as \"handle\" for lane sensor " << path);
        mScan.request.handle = getHandle(name, car.index());
        srv = ros::NodeHandle().serviceClient<simRosGetVisionSensorDepthBuffer>("/vrep/simRosGetVisionSensorDepthBuffer", true);
        update();
      }

      virtual bool update() {
        if(!srv.call(mScan)) {
          ROS_ERROR_STREAM("Error on road scan retrieval of RoadSensor " << mScan.request.handle);
          return false;
        }
        size_t start, stop;
        bool left = false;
        size_t resolution = mScan.response.resolution[0]*mScan.response.resolution[1];
        for( size_t i = 0; i < resolution; i++ ) {
          if(mScan.response.buffer[i] < roadTreshold ) {
            stop = i;
            left = true;
          }
          if ( !left )
            start = i;
        }
        mValue = ( (float)( start + stop ) / resolution ) - 1;
        ROS_DEBUG_STREAM(*this);
        return true;
      }

      virtual void print(ostream& o) const {
        o << "Lane Position of sensor " << mScan.request.handle << ": " << value();
      }
  };

  DataPtr createData(const string& path, Car& car) {
    string type;
    ROS_INFO_STREAM("Qurying sensor type: " << path+"/type");
    if(ros::param::get(path+"/type", type)) {
        ROS_INFO_STREAM("Got sensor type: " << type);
        if(type == "laneSensor")
          return DataPtr(new LaneSensor(path, car));
        if(type == "command")
          return DataPtr(new Command(path, car));
        if(type == "float")
          return DataPtr(new Float(path ,car));
    }
    return DataPtr();
  }
}
