#include "Car.h"

#include <aseia_car_sim/RegisterCar.h>
#include <aseia_car_sim/UpdateCar.h>

#include <ros/ros.h>

#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosGetVisionSensorDepthBuffer.h>
#include <vrep_common/simRosGetObjectPose.h>
#include <vrep_common/simRosSetJointTargetPosition.h>
#include <vrep_common/simRosSetJointTargetVelocity.h>

#include <Eigen/Core>

#include <cmath>
#include <atomic>
#include <mutex>
#include <thread>

namespace car {

  using namespace std;
  using namespace vrep_common;
  using namespace aseia_car_sim;
  using HandleMap = std::unordered_map<string, unsigned int>;

  template<size_t n>
  class FloatArray  : public Data, public Eigen::Array<float, n, 1>{
    private:
      using Base = Eigen::Array<float, n, 1>;
    public:
      FloatArray(HandleMap& handles, float value=0.0f, bool input=false, bool output=false)
        : Data("float"+to_string(n), input, output),
          Base(Base::Constant(value))
      {}
      virtual void print(ostream& o) const { o << (const Base&)(*this); }
  };

  class Command : public FloatArray<2> {
    private:
      ros::NodeHandle mNh;
      ros::ServiceClient mPoseSrv, mVelSrv, mPosSrv;
      const int mLeftSteer, mRightSteer, mLeftMotor, mRightMotor;
      const float mWidth, mLength;
      using Base = FloatArray<2>;
      using A2 = Eigen::Array2f;
      using V3 = Eigen::Vector3f;

      V3 getPosition(int handle, int reference) {
        simRosGetObjectPose pose;
        pose.request.handle = handle;
        pose.request.relativeToObjectHandle = reference;
        mPoseSrv.call(pose);
        if( pose.response.result == -1 )
          ROS_ERROR_STREAM("Error getting pose of object with handle " << handle);
        float x,y,z;
        x = pose.response.pose.pose.position.x;
        y = pose.response.pose.pose.position.y;
        z = pose.response.pose.pose.position.z;
        return V3(x, y, z);
      }

      void setSpeeds(A2 data) {
        simRosSetJointTargetVelocity velocity;
        velocity.request.handle = mLeftMotor;
        velocity.request.targetVelocity = data(0);
        if( !mVelSrv.call(velocity) || velocity.response.result == -1 )
          ROS_ERROR_STREAM("Error setting velocity of joint handle " << mLeftMotor);
        velocity.request.handle = mRightMotor;
        velocity.request.targetVelocity = data(1);
        if( !mVelSrv.call(velocity) || velocity.response.result == -1 )
          ROS_ERROR_STREAM("Error setting velocity of joint handle " << mRightMotor);
      }

      void setAngles(A2 data) {
        simRosSetJointTargetPosition angle;
        angle.request.handle = mLeftSteer;
        angle.request.targetPosition = data(0);
        if( !mPosSrv.call(angle) || angle.response.result == -1 )
          ROS_ERROR_STREAM("Error setting angle of joint handle " << mLeftMotor);
        angle.request.handle = mRightSteer;
        angle.request.targetPosition = data(1);
        if( !mPosSrv.call(angle) || angle.response.result == -1 )
          ROS_ERROR_STREAM("Error setting angle of joint handle " << mRightMotor);
      }

    public:
      static const size_t steerIndex = 0;
      static const size_t speedIndex = 1;
      Command(HandleMap& handles, float value=0.0f)
        : Base(handles, value, false, true),
          mPoseSrv(mNh.serviceClient<simRosGetObjectPose>("/vrep/simRosGetObjectPose", true)),
          mVelSrv(mNh.serviceClient<simRosSetJointTargetVelocity>("/vrep/simRosSetJointTargetVelocity", true)),
          mPosSrv(mNh.serviceClient<simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition", true)),
          mLeftSteer(handles["lSteer"]), mRightSteer(handles["rSteer"]),
          mLeftMotor(handles["lMotor"]), mRightMotor(handles["rMotor"]),
          mWidth((getPosition(handles["lMotor"], handles["car"])-getPosition(handles["rMotor"], handles["car"])).norm()/2),
          mLength((getPosition(handles["lMotor"], handles["car"])-getPosition(handles["blAxis"], handles["car"])).norm())
      {}
      float steer() const { return this->operator()(steerIndex); }
      void steer(float value) { this->operator()(steerIndex) = value; }
      float speed() const { return this->operator()(speedIndex); }
      void speed(float value) { this->operator()(speedIndex) = value; }
      void print(ostream& o) const { o << "Command: "; this->Base::print(o); }
      void update() {
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
        setSpeeds(speeds);
        setAngles(angles);
        ROS_DEBUG_STREAM("Speed: " << speeds);
        ROS_DEBUG_STREAM("Angle: " << angles);
        ROS_DEBUG_STREAM(*this);
      }
  };

  class Float : public Data {
    protected:
      float mValue;
    public:
      Float(HandleMap& handles, float value = 0.0f, bool input = false, bool output = false)
        : Data("float", input, output),
          mValue(value) {}
      float value() const { return mValue; }
      void value(float v) { mValue = v; }
      void print(ostream& o) const { o << type() << " " << mValue; }
  };

  class LanePos : public Float {
    private:
      simRosGetVisionSensorDepthBuffer mScan;
      float mPos;
      ros::ServiceClient srv;
      const float roadTreshold = 0.9;
    public:
      using Resolution = pair<unsigned int, unsigned int>;
      LanePos(HandleMap& handles, float value=0.0f)
        : Float(handles, value, true)
      {
        mScan.request.handle = handles["laneScanner"];
        ros::NodeHandle nh;
        do {
          srv = nh.serviceClient<simRosGetVisionSensorDepthBuffer>("/vrep/simRosGetVisionSensorDepthBuffer", true);
        } while(!srv.exists());
        update();
      }

      virtual void update() {
        if(!srv.call(mScan))
          ROS_ERROR_STREAM("Error on road scan retrieval of RoadSensor " << mScan.request.handle);
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
      }

      virtual void print(ostream& o) const {
        o << "Lane Position of sensor " << mScan.request.handle << ": " << value();
      }

  };

  class LaneControl : public Controller {
    public:
      LaneControl(): Controller("lane", 0) {
      }

      std::string dataType() const { return "float"; }

      virtual void operator()(DataMap& data) {
        const Data* inPtr = data["lanePos"].get();
        const Data* refPtr = data["laneRef"].get();
              Data* cmdPtr = data["command"].get();
        if( !inPtr || !refPtr || !cmdPtr ) {
          ROS_ERROR_STREAM("Lane controller got not enough data");
          return;
        }
        const Float&   in  = dynamic_cast<const Float&  >(*inPtr);
        const Float&   ref = dynamic_cast<const Float&  >(*refPtr);
              Command& cmd = dynamic_cast<Command&>(*cmdPtr);
        cmd.steer( ( in.value() - ref.value() ) * 4 );
        ROS_DEBUG_STREAM(*this);
      }

      virtual void print(ostream& o) const {
        o << type() << ": (lanePos - laneRef) -> command";
      }
  };

  class CruiseControl : public Controller {
    public:
      CruiseControl() : Controller("cruise", 1) {}

      virtual void operator()(DataMap& data) {
        const Data* refPtr = data["speedRef"].get();
              Data* cmdPtr = data["command"].get();
        if( !refPtr || !cmdPtr ) {
          ROS_ERROR_STREAM("Lane controller got not enough data");
          return;
        }
        const Float&   ref = dynamic_cast<const Float&  >(*refPtr);
              Command& cmd = dynamic_cast<Command&>(*cmdPtr);
        cmd.speed( ref.value() );
        ROS_DEBUG_STREAM(*this);

      }

      virtual void print(ostream& o) const {
        o << type() << ": (speedRef) -> command";
      }
  };

  class Car {
    private:
      using HandleMap  = unordered_map< string, unsigned int >;
      using ControlVec = vector< ControllerPtr >;
      using DataElem = DataMap::value_type;
      DataMap mData;
      HandleMap mHandles;
      ControlVec mControl;
      mutex mTrigger;
      atomic<bool> mDone;
      bool mAlive;
      const std::string mBaseName;
      const size_t mIndex;
      thread mUpdateThread;
      ros::ServiceServer mUpdateSrv;

      int getHandle(const string& objectName) {
        ros::NodeHandle nh;
        simRosGetObjectHandle handle;
        handle.request.objectName = objectName+"#"+to_string(mIndex);
        nh.serviceClient< simRosGetObjectHandle >( "/vrep/simRosGetObjectHandle").call(handle);
        ROS_INFO_STREAM(handle.request.objectName << ": " << handle.response.handle);
        return handle.response.handle;
      }
    public:


      bool alive() const { return mAlive; }

      void addHandle(const string& simName, const string& handleName) {
        int handle = getHandle(simName);
        if(handle != -1)
          mHandles[handleName] = handle;
      }

      void addController(const string& ctrlType) {
        ControllerPtr ctrlPtr;
        if(ctrlType == "lane")
          ctrlPtr.reset(new LaneControl());
        if(ctrlType == "cruise")
          ctrlPtr.reset(new CruiseControl());
        if(ctrlPtr) {
          mControl.emplace_back(std::move(ctrlPtr));
          ROS_INFO_STREAM("Added " << *mControl.back());
          auto cmp = [](const ControllerPtr& a, const ControllerPtr& b){ return *a < *b; };
          sort(mControl.begin(), mControl.end(), cmp);
        }
      }

      void addData(const string& dataName, const string& dataType, float value=0.0f) {
        DataPtr dataPtr;
        if(dataType == "lanePosition")
          dataPtr.reset(new LanePos(mHandles, value));
        if(dataType == "command")
          dataPtr.reset(new Command(mHandles, value));
        if(dataType == "float")
          dataPtr.reset(new Float(mHandles, value));
        if(dataPtr) {
          pair<DataMap::iterator, bool> res = mData.emplace(dataName, std::move(dataPtr));
          if(res.second)
            ROS_INFO_STREAM("Added " << res.first->first << ": " << *res.first->second);
        }
      }

      void update() {
        lock_guard<mutex> lock(mTrigger);
        mDone = false;
        for( DataElem& data : mData )
          if( data.second && data.second->isInput() )
            data.second->update();

        for( ControllerPtr& ctrlPtr : mControl )
          if( ctrlPtr )
            (*ctrlPtr)(mData);

        for( DataElem& data : mData )
          if( data.second && data.second->isOutput() )
            data.second->update();
        mDone = true;
      }

      string name() const { return mBaseName + to_string(mIndex); }

      bool handleUpdate(UpdateCar::Request& req, UpdateCar::Response& res) {
        switch(req.command) {
          case(UpdateCar::Request::trigger): mTrigger.unlock();
                                             res.alive = mAlive;
                                             break;
          case(UpdateCar::Request::wait)   : res.done = mDone.load();
                                             break;
        }
        res.alive = mAlive;
        return true;
      }

      Car(const std::string& baseName, std::size_t i, const std::string& simName)
        : mDone(true),
          mAlive(true),
          mBaseName(baseName),
          mIndex(i)
      {
        ros::NodeHandle nh;
        mTrigger.lock();
        mUpdateThread=std::move(thread([this](){ while(ros::ok()) this->update(); }));
        mUpdateSrv = nh.advertiseService(name()+"/update", &Car::handleUpdate, this);
        RegisterCar reg;
        reg.request.name=name();
        ros::ServiceClient srv;
        do {
          srv=ros::NodeHandle().serviceClient<RegisterCar>(simName+"/registerCar");
        }while(!srv.exists());
        if(! srv.call(reg) || !reg.response.result)
          ROS_FATAL_STREAM("Could not register car "<<name()<< " with simulation " << simName);
        addHandle("RoadSensor"    , "laneScanner");
        addHandle("SteeringLeft"  , "lSteer");
        addHandle("SteeringRight" , "rSteer");
        addHandle("MotorLeft"     , "lMotor");
        addHandle("MotorRight"    , "rMotor");
        addHandle("FreeAxisLeft"  , "blAxis");
        addHandle("Car"           , "car");
        addData("laneRef" , "float", 0.0f);
        addData("speedRef", "float", 10.0f);
        addData("lanePos" , "lanePosition");
        addData("command" , "command");
        addController("lane");
        addController("cruise");
      }

      ~Car() {
        mTrigger.unlock();
        mUpdateThread.join();
      }
  };
}

using namespace car;

int main(int argc, char** argv) {
  ros::init(argc, argv, "car");
  ros::NodeHandle nh;
  Car car("Car", 0, "simulation");
  while(ros::ok())
    ros::spin();
  return 0;
}
