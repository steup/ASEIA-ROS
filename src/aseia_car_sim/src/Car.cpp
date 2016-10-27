#include "Car.h"

#include <ros/ros.h>

#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosEnablePublisher.h>
#include <v_repConst.h>

namespace car {

  using namespace std;
  using namespace vrep_common;

  class Command : public Data {
    private:
      float mSpeed, mAngle;
      const int mLeftSteer, mRightSteer, mLeftMotor, mRightMotor;
    public:
      Command(int lSteer, int rSteer, int lMotor, int rMotor)
        : mSpeed(0.0f), mAngle(0.0f),
          mLeftSteer(lSteer), mRightSteer(rSteer),
          mLeftMotor(lMotor), mRightMotor(rMotor)
      {}
      void angle(float angle) { mAngle = angle; }
      void speed(float speed) { mSpeed = speed; }
      float angle() const { return mAngle; }
      float speed() const { return mSpeed; }
      virtual void update() {
      }
  };

  class RoadScan : public Data, public vector<float> {
    private:
      int mSensor;
    public:
      RoadScan(int sensorHandle) : mSensor(sensorHandle) {

      }
      virtual void update() {

      }
  };

  class LaneControl : public Controller {
    private:
      Data mData;
    public:
      LaneControl() {}

      void reference(const Data& ref) {

      }

      const Data& reference() const {
        return mData;
      }

      void operator()(DataMap& data) {

      }
  };

  ControllerPtr ctrlFactory(const string& name) {
    if(name == "lane")
      return ControllerPtr(new LaneControl());
    return ControllerPtr();
  }

  Car::Car(size_t i)  : mAlive(true), mIndex(i) {
    ros::NodeHandle nh;
    simRosGetObjectHandle handle;
    handle.request.objectName = name();
    ros::ServiceClient getHandle = nh.serviceClient< simRosGetObjectHandle >( "/vrep/simRosGetObjectHandle", true );
   /* getHandle.call( handle );
    mCar = handle.response.handle;
    ROS_INFO_STREAM("" << name() << ": " << mHandle);*/

    handle.request.objectName = "RoadSensor#"+to_string(mIndex);
    getHandle.call( handle );
    ROS_INFO_STREAM("RoadSensor#" << to_string(mIndex) << ": " << handle.response.handle);
    mData.emplace(Data::Type::roadScan, DataPtr(new RoadScan(handle.response.handle)));
    int leftSteer, rightSteer, leftMotor, rightMotor;
    mData.emplace(Data::Type::command,  DataPtr(new Command(leftSteer, rightSteer, leftMotor, rightMotor)));

   /* simRosEnablePublisher createPub;
    createPub.request.topicName = frame();
    createPub.request.queueSize = 1;
    createPub.request.streamCmd = simros_strmcmd_get_object_pose;
    createPub.request.auxInt1   = getHandle.response.handle;
    createPub.request.auxInt2   = -1;
    createPub.request.auxString = frame();
    nh.serviceClient< simRosEnablePublisher >( "/vrep/simRosEnablePublisher" ).call( createPub );
    if( !createPub.response.effectiveTopicName.empty() ) {
      mPubs.push_back( createPub.response.effectiveTopicName );
      ROS_INFO_STREAM("Succesfully created pose publisher of " << name() << " to topic " << frame());
    }*/
  }

  void Car::update() {
    for( DataElem& data : mData )
      if( data.first < Data::Type::out && data.first > Data::Type::in )
        data.second->update();

    for( ControllerPtr& ctrlPtr : mControl )
      if( ctrlPtr )
        (*ctrlPtr)(mData);

    for( DataElem& data : mData )
      if( data.first > Data::Type::out )
        data.second->update();
  }

  Data Car::speed() const {
    return Data();
  }

  Data Car::angle() const {
    return Data();
  }

  string Car::name() const { return "Car#" + to_string( mIndex ); }
  string Car::frame() const { return "/car/" + to_string( mIndex ); }
}
