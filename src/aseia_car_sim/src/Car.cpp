#include "Car.h"

#include <ros/ros.h>

#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosEnablePublisher.h>
#include <v_repConst.h>

namespace car {

  using namespace std;
  using namespace vrep_common;

  class RoadScan : public Data {

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
    simRosGetObjectHandle getHandle;
    getHandle.request.objectName = name();
    nh.serviceClient< simRosGetObjectHandle >( "/vrep/simRosGetObjectHandle" ).call( getHandle );
    ROS_INFO_STREAM("" << name() << ": " << getHandle.response.handle);

    simRosEnablePublisher createPub;
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
    }
  }

  void Car::feed(const Data& data) {

  }

  void Car::update() {

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
