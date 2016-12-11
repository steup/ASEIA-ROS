#include "Data.h"
#include "DataException.h"
#include "Car.h"

#include <ros/ros.h>

#include <vrep_common/simRosGetVisionSensorDepthBuffer.h>
#include <vrep_common/simRosReadProximitySensor.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosGetObjectPose.h>
#include <vrep_common/simRosSetJointTargetPosition.h>
#include <vrep_common/simRosSetJointTargetVelocity.h>

#include <Eigen/Core>
#include <cmath>

namespace car {

  using namespace std;
  using namespace vrep_common;
  using namespace Eigen;

  using A2 = Array2f;
  using V3 = Vector3f;
  using MV3 = Map<Vector3f>;

  static ros::ServiceClient handleSrv, poseSrv, depthSrv, jointPosSrv, jointVelSrv;

  template<typename ServiceType>
  void checkedCall(ros::ServiceClient& srv, ServiceType& data){
    if( !srv.call(data)) {
      if(!srv.exists())
        throw DataException(DataException::Reason::vrepGone);
      else
        throw DataException(DataException::Reason::serviceGone);
    }
    if(data.response.result==-1)
        throw DataException(DataException::Reason::badResult);
  }

  void checkedCall(ros::ServiceClient& srv, simRosGetObjectHandle& data){
    if( !srv.call(data)) {
      if(!srv.exists())
        throw DataException(DataException::Reason::vrepGone);
      else
        throw DataException(DataException::Reason::serviceGone);
    }
    if(data.response.handle==-1)
        throw DataException(DataException::Reason::badResult);
  }


  static int getHandle(const string& objectName, int index = -1) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosGetObjectHandle >( "/vrep/simRosGetObjectHandle", true);
    simRosGetObjectHandle handle;
    handle.request.objectName = objectName;
    if(index != -1)
      handle.request.objectName += "#"+to_string(index);
    checkedCall(srv, handle);
    if(handle.response.handle == -1)
      ROS_ERROR_STREAM("No V-REP object with name "<<handle.request.objectName<<" found");
    return handle.response.handle;
  }

  static V3 getPosition(int handle, int reference) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosGetObjectPose >( "/vrep/simRosGetObjectPose", true);
    simRosGetObjectPose pose;
    pose.request.handle = handle;
    pose.request.relativeToObjectHandle = reference;
    checkedCall(srv, pose);
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
    checkedCall(srv, data);
    return true;
  }

  static bool setSpeed(int handle, float vel) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosSetJointTargetVelocity >( "/vrep/simRosSetJointTargetVelocity", true);
    simRosSetJointTargetVelocity data;
    data.request.handle = handle;
    data.request.targetVelocity = vel;
    checkedCall(srv,  data);
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

  static void getVisionDepthData(simRosGetVisionSensorDepthBuffer& buffer) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient<simRosGetVisionSensorDepthBuffer>("/vrep/simRosGetVisionSensorDepthBuffer", true);
    checkedCall(srv, buffer);
  }


  class LaneSensor : public Float {
    private:
      simRosGetVisionSensorDepthBuffer mScan;
      float mPos;
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
        update();
      }

      virtual bool update() {
        getVisionDepthData(mScan);
        size_t start=0, stop=0;
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
        o << "Lane Position of sensor " << mScan.request.handle << ": " << mValue;
      }
  };

  class VisionDistanceSensor: public Float {
    private:
      simRosGetVisionSensorDepthBuffer mScan;
      using Resolution = pair<unsigned int, unsigned int>;
    public:
      VisionDistanceSensor(const std::string& path, const Car& car)
        : Float(path, car, true)
      {
          string name;
          if( !ros::param::get(path+"/handle", name) )
            ROS_ERROR_STREAM("No V-REP sensor name supplied as \"handle\" for vision depth sensor " << path);
          mScan.request.handle = getHandle(name, car.index());
          update();
      }
        virtual bool update() {
          getVisionDepthData(mScan);
          mValue = numeric_limits<float>::infinity();
          for(const float currentValue : mScan.response.buffer)
            if(currentValue < mValue)
              mValue = currentValue;
          ROS_DEBUG_STREAM(*this);
          return mValue != numeric_limits<float>::infinity();
        }

        virtual void print(ostream& o) const {
          o << "Vision Distance Sensor value: " << mScan.request.handle << ": " << mValue;
        }
  };

  static void getProximityDistance(simRosReadProximitySensor& buffer) {
      static ros::ServiceClient srv = ros::NodeHandle().serviceClient<simRosReadProximitySensor>("/vrep/simRosReadProximitySensor", true);
      checkedCall(srv, buffer);
  }

  class ProximityDistanceSensor: public Float {
    private:
      simRosReadProximitySensor mData;
      using Resolution = pair<unsigned int, unsigned int>;
    public:
      ProximityDistanceSensor(const std::string& path, const Car& car)
        : Float(path, car, true)
      {
          string name;
          if( !ros::param::get(path+"/handle", name) )
            ROS_ERROR_STREAM("No V-REP sensor name supplied as \"handle\" for distance sensor " << path);
          mData.request.handle = getHandle(name, car.index());
          update();
      }
      virtual bool update() {
        getProximityDistance(mData);
        MV3 v(mData.response.detectedPoint.data());
        mValue = v.norm();
        ROS_DEBUG_STREAM(*this);
        return mData.response.result!=-1;
      }

      virtual void print(ostream& o) const {
        o << "Proximity Distance Sensor value: " << mData.request.handle << ": " << mValue;
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
        if(type == "visionDistance")
          return DataPtr(new VisionDistanceSensor(path ,car));
        if(type == "proxDistance")
          return DataPtr(new ProximityDistanceSensor(path ,car));
    }
    return DataPtr();
  }
}
