#include "Car.h"
#include "Data.h"
#include "Controller.h"
#include "VRepWrapper.h"

#include <aseia_car_sim/RegisterCar.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <boost/algorithm/string.hpp>

#include <cmath>
#include <atomic>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <condition_variable>

namespace car {

  using namespace std;
  using namespace aseia_car_sim;
  using namespace Eigen;
  using namespace vrep;

  using A2 = Array2f;
  using V3 = Vector3f;

  class CarImpl : public Car {
    private:
      using DataMap = unordered_map<string, DataPtr>;
      using ControlVec = vector< ControllerPtr >;
      using DataElem = DataMap::value_type;
      DataMap mRef, mSensor, mAct;
      ControlVec mControl;
      ros::Subscriber mUpdateSub;
      ros::Publisher  mDonePub;
      atomic<bool> mDone;
      mutex mMutex;
      condition_variable mCond;
      AngularJoint mLeftSteer, mRightSteer;
      VelocityJoint mLeftMotor, mRightMotor;
      Object mCar;
      const float mWidth, mLength;

    public:

      bool addController(const string& ctrlName) {
        string key;
        if(!ros::param::search("controllers", key))
          return false;
        ControllerPtr ctrlPtr = createController(key+"/"+ctrlName, *this);
        if(ctrlPtr) {
          mControl.emplace_back(std::move(ctrlPtr));
          ROS_INFO_STREAM("Added " << *mControl.back());
          auto cmp = [](const ControllerPtr& a, const ControllerPtr& b){ return *a < *b; };
          sort(mControl.begin(), mControl.end(), cmp);
          return true;
        }else
          ROS_ERROR_STREAM("Invalid controller "<< key << "/" << ctrlName);
        return false;
      }

      bool addSensor(const string& sensorName) {
        string key;
        if(!ros::param::search("sensors", key))
          return false;
        DataPtr dataPtr=createData(key+"/"+sensorName, *this);
        if(dataPtr) {
          pair<DataMap::iterator, bool> res = mSensor.emplace(sensorName, std::move(dataPtr));
          if(res.second)
            ROS_INFO_STREAM("Added " << res.first->first << ": " << *res.first->second);
          return true;
        }else
          ROS_ERROR_STREAM("Invalid sensor "<< key << "/" << sensorName);

        return false;
      }

      bool addActuator(const string& actName) {
        string key;
        if(!ros::param::search("actuators", key))
          return false;
        DataPtr dataPtr=createData(key+"/"+actName, *this);
        if(dataPtr) {
          pair<DataMap::iterator, bool> res = mAct.emplace(actName, std::move(dataPtr));
          if(res.second)
            ROS_INFO_STREAM("Added " << res.first->first << ": " << *res.first->second);
          return true;
        }else
          ROS_ERROR_STREAM("Invalid actuator "<< key << "/" << actName);

        return false;
      }

      bool addRef(const string& refName) {
        string key;
        if(!ros::param::search("references", key))
          return false;
        DataPtr dataPtr=createData(key+"/"+refName, *this);
        if(dataPtr) {
          pair<DataMap::iterator, bool> res = mRef.emplace(refName, std::move(dataPtr));
          float value;
          ros::param::get(key+"/"+refName+"/value", value);
          dynamic_cast<Float&>(*res.first->second).value = value;
          if(res.second)
            ROS_INFO_STREAM("Added " << res.first->first << ": " << *res.first->second);
          return true;
        }else
          ROS_ERROR_STREAM("Invalid reference "<< key << "/" << refName);

        return false;
      }

      string name() const { return ros::this_node::getName(); }

      void handleUpdate(const std_msgs::BoolConstPtr& kill) {
        if(!kill->data)
          ros::shutdown();
        else {
          ROS_DEBUG_STREAM("Updating car " << name());
          try {
            if(mAlive) {
              for( DataElem& data : mSensor )
                if( data.second && data.second->isInput() )
                  if( !data.second->update() )
                    mAlive = false;

              for( ControllerPtr& ctrlPtr : mControl )
                if( ctrlPtr )
                  if( !(*ctrlPtr)() )
                    mAlive = false;


              for( DataElem& data : mAct )
                if( data.second && data.second->isOutput() )
                  if( !data.second->update() )
                    mAlive = false;

              const Float& speed = dynamic_cast<const Float&>(*mAct["speed"]);
              const Float& steer = dynamic_cast<const Float&>(*mAct["steer"]);
              A2 speeds = A2::Constant(speed)*2/0.76;
              A2 angles = A2::Zero();
              if(steer > 0.0001 || steer < -0.0001) {
                float r = mLength * tanf(M_PI_2 - steer);
                float a = mWidth / ( r * r + mLength * mLength );
                A2 s(copysign(steer, 1.0f), -copysign(steer, 1.0f));
                speeds  *= (s * r * a + mWidth * a + 1).sqrt();

                //compute left and right angle of wheels dependent on radius of curve
                angles = ( mLength / ( s * mWidth + r) ).unaryExpr(ptr_fun(atanf));
              }
              ROS_DEBUG_STREAM("Speed: " << speeds);
              ROS_DEBUG_STREAM("Angle: " << angles);
              mLeftSteer.angle(angles(0));
              mRightSteer.angle(angles(1));
              mLeftMotor.velocity(speeds(0));
              mRightMotor.velocity(speeds(1));
            }
          }
          catch(const Exception& e) {
            ROS_ERROR_STREAM(e.what());
            if(e.reason==Exception::Reason::vrepGone || e.reason == Exception::Reason::serviceGone)
              ros::shutdown();
          }
        }
        std_msgs::Bool msg;
        msg.data = mAlive;
        mDonePub.publish(msg);
        ROS_DEBUG_STREAM("Updating car " << name() << "done");
      }

      CarImpl(std::size_t i, const std::string& simName)
        : Car(i),
          mDone(false),
          mLeftSteer("SteeringLeft", index()),
          mRightSteer("SteeringRight", index()),
          mLeftMotor("MotorLeft", index()),
          mRightMotor("MotorRight", index()),
          mCar("Car", index()),
          mWidth( (mLeftSteer.position(mCar) - mRightMotor.position(mCar)).norm()/2 ),
          mLength( (mLeftSteer.position(mCar) - Object("FreeAxisLeft", index()).position(mCar)).norm() )
      {
        ros::NodeHandle nh;
        mUpdateSub = nh.subscribe("/cars/update", 1, &CarImpl::handleUpdate, this);
        mDonePub   = nh.advertise<std_msgs::Bool>(name()+"/done", 1);
        RegisterCar reg;
        reg.request.name=name();
        ros::ServiceClient srv = ros::NodeHandle().serviceClient<RegisterCar>(simName+"/registerCar");
        srv.waitForExistence();
        if(! srv.call(reg) || !reg.response.result)
          ROS_FATAL_STREAM("Could not register car "<<name()<< " with simulation " << simName);
      }

      const Data* getReference(const string& name) const {
        DataMap::const_iterator iter = mRef.find(name);
        if(iter != mRef.end())
          return iter->second.get();
        else
          return nullptr;
      }

      const Data* getSensor(const string& name) const {
        DataMap::const_iterator iter = mSensor.find(name);
        if(iter != mSensor.end())
          return iter->second.get();
        else
          return nullptr;
      }

      Data* getActuator(const string& name) {
        DataMap::iterator iter = mAct.find(name);
        if(iter != mAct.end())
          return iter->second.get();
        else
          return nullptr;
      }
  };
}

using namespace car;
using namespace boost;

int main(int argc, char** argv) {
  ros::init(argc, argv, "car");
  ros::NodeHandle nh("~");
  int i;
  string simName;
  nh.getParam("index", i);
  if(i<0) {
    ROS_ERROR_STREAM("Invalid car index " << i);
    return -1;
  }
  string simKey;
  while(!nh.searchParam("simName", simKey));
  nh.getParam(simKey, simName);
  if(simName.empty()) {
    ROS_FATAL_STREAM("Invalid simulation " << simName);
    return -1;
  }
  CarImpl car(i, simName);
  {
    string temp;
    vector<string> splitted;
    if( nh.getParam("sens", temp) && !temp.empty() ) {
      ROS_INFO_STREAM("sensors of car " <<i<<": "<<temp);
      split(splitted, temp, is_any_of(", "), token_compress_on);
      for(const auto& sensor : splitted)
        if( !car.addSensor(sensor) )
          ROS_ERROR_STREAM("Error adding sensor " << sensor);
    }

    if( nh.getParam("acts", temp) && !temp.empty() ) {
      ROS_INFO_STREAM("Actuators of car " <<i<<": "<<temp);
      split(splitted, temp, is_any_of(", "), token_compress_on);
      for(const string& act : splitted)
        if( !car.addActuator(act) )
          ROS_ERROR_STREAM("Error adding actuator " << act;);
    }

    if( nh.getParam("ctrls", temp) && !temp.empty() ) {
      ROS_INFO_STREAM("controllers of car " <<i<<": "<<temp);
      split(splitted, temp, is_any_of(", "), token_compress_on);
      for(const string& ctrl : splitted)
        if( !car.addController(ctrl) )
          ROS_ERROR_STREAM("Error adding controller " << ctrl;);
    }

    if( nh.getParam("refs", temp) && !temp.empty() ) {
      ROS_INFO_STREAM("references of car " <<i<<": "<<temp);
      split(splitted, temp, is_any_of(", "), token_compress_on);
      for(const string& ref: splitted)
        if( !car.addRef(ref) )
          ROS_ERROR_STREAM("Error adding reference " << ref;);
    }
  }
  while(ros::ok())
    ros::spin();
  return 0;
}
