#include "VRepWrapper.h"

#include <ros/ros.h>

#include <vrep_common/simRosGetVisionSensorDepthBuffer.h>
#include <vrep_common/simRosReadProximitySensor.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosGetObjectPose.h>
#include <vrep_common/simRosSetJointTargetPosition.h>
#include <vrep_common/simRosSetJointTargetVelocity.h>

namespace vrep {

using namespace vrep_common;
using namespace std;

const char* Exception::what() const throw() {
  switch(reason) {
    case(Reason::vrepGone)   : return "V-Rep is gone";
    case(Reason::badResult)  : return "Service call failed with bad result";
    case(Reason::serviceGone): return "Persistent service is gone";
    default                  : return "Unknown error";
  }
}

const char* ExtendedException::what() const throw() {
  return ( string(Exception::what()) + mOs.str() ).c_str();
}

template<typename ServiceType>
void checkedCall(ros::ServiceClient& srv, ServiceType& data){
  if( !srv.call(data)) {
    if(!srv.exists())
      throw Exception(Exception::Reason::vrepGone);
    else
      throw Exception(Exception::Reason::serviceGone);
  }
  if(data.response.result==-1)
      throw Exception(Exception::Reason::badResult);
}

void checkedCall(ros::ServiceClient& srv, simRosGetObjectHandle& data){
  if( !srv.call(data)) {
    if(!srv.exists())
      throw Exception(Exception::Reason::vrepGone);
    else
      throw Exception(Exception::Reason::serviceGone);
  }
  if(data.response.handle==-1)
      throw Exception(Exception::Reason::badResult);
}


int getHandle(const string& objectName, int index = -1) {
  static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosGetObjectHandle >( "/vrep/simRosGetObjectHandle", true);
  simRosGetObjectHandle handle;
  handle.request.objectName = objectName;
  if(index != -1)
    handle.request.objectName += "#"+to_string(index);
  try {
    checkedCall(srv, handle);
  } catch(Exception& e) {
      throw ExtendedException(e) << "No object with name" << handle.request.objectName << " - " << __FILE__ << ":" << __LINE__;
  }
  if(handle.response.handle == -1)
    ROS_ERROR_STREAM("No V-REP object with name "<<handle.request.objectName<<" found");
  return handle.response.handle;
}

static void setAngle(int handle, float angle) {
  static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosSetJointTargetPosition >( "/vrep/simRosSetJointTargetPosition", true);
  simRosSetJointTargetPosition data;
  data.request.handle = handle;
  data.request.targetPosition = angle;
  try {
    checkedCall(srv, data);
  } catch(Exception& e) {
      throw ExtendedException(e) << "Cannot set angle of rotary joint " << data.request.handle << " to " << data.request.targetPosition << " - " << __FILE__ << ":" << __LINE__;
  }
}

static void setSpeed(int handle, float vel) {
  static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosSetJointTargetVelocity >( "/vrep/simRosSetJointTargetVelocity", true);
  simRosSetJointTargetVelocity data;
  data.request.handle = handle;
  data.request.targetVelocity = vel;
  try {
    checkedCall(srv, data);
  } catch(Exception& e) {
      throw ExtendedException(e) << "Cannot set velocity of rotary joint " << data.request.handle << " to " << data.request.targetVelocity << " - " << __FILE__ << ":" << __LINE__;
  }
}

static Object::Position getPosition(int handle, int reference) {
  static ros::ServiceClient srv = ros::NodeHandle().serviceClient< simRosGetObjectPose >( "/vrep/simRosGetObjectPose", true);
  simRosGetObjectPose pose;
  pose.request.handle = handle;
  pose.request.relativeToObjectHandle = reference;
  try {
    checkedCall(srv, pose);
  } catch(Exception& e) {
      throw ExtendedException(e) << "Cannot get position of object " << pose.request.handle << " - " << __FILE__ << ":" << __LINE__;
  }
  float x,y,z;
  x = pose.response.pose.pose.position.x;
  y = pose.response.pose.pose.position.y;
  z = pose.response.pose.pose.position.z;
  return Object::Position(x, y, z);
}

static void getVisionDepthData(simRosGetVisionSensorDepthBuffer& buffer) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient<simRosGetVisionSensorDepthBuffer>("/vrep/simRosGetVisionSensorDepthBuffer", true);
  try {
    checkedCall(srv, buffer);
  } catch(Exception& e) {
      throw ExtendedException(e) << "Cannot get depth buffer from vision sensor " << buffer.request.handle << " - " << __FILE__ << ":" << __LINE__;
  }
}

static void getProximityDistance(simRosReadProximitySensor& buffer) {
    static ros::ServiceClient srv = ros::NodeHandle().serviceClient<simRosReadProximitySensor>("/vrep/simRosReadProximitySensor", true);
  try {
    checkedCall(srv, buffer);
  } catch(Exception& e) {
      throw ExtendedException(e) << "Cannot get distance data from from proximity sensor " << buffer.request.handle << " - " << __FILE__ << ":" << __LINE__;
  }
}

  Object::Object(string name, int index) throw(Exception)
    : handle(getHandle(name, index))
  {}

  Object::Position Object::position(const Object& reference) const throw(Exception) {
    return getPosition(handle, reference.handle);
  }

  AngularJoint::AngularJoint(std::string name, int index) throw(Exception)
    : Object(name, index)
  {}

  void AngularJoint::angle(float angle) throw(Exception) {
    mAngle = angle;
    setAngle(handle, angle);
  }

  VelocityJoint::VelocityJoint(std::string name, int index) throw(Exception)
    : Object(name, index)
  {}

  void VelocityJoint::velocity(float velocity) throw(Exception) {
    mVelocity = velocity;
    setSpeed(handle, mVelocity);
  }

  ProximitySensor::ProximitySensor(std::string name, int index) throw(Exception)
    : Object(name, index)
  {}

  float ProximitySensor::distance() const throw(Exception) {
    simRosReadProximitySensor buffer;
    buffer.request.handle = handle;
    getProximityDistance(buffer);
    Object::Position v(buffer.response.detectedPoint.data());
    return v.norm();
  }

  using VDS = VisionDepthSensor;

  struct VDSImpl {
    VDS& interface;
    simRosGetVisionSensorDepthBuffer buffer;

    VDSImpl(VDS& interface) : interface(interface) {
      buffer.request.handle = interface.handle;
    }

    const VDS::Resolution resolution() {
      getVisionDepthData(buffer);
      return VDS::Resolution(buffer.response.resolution.data());
    }

    const VDS::Distances distances() {
      getVisionDepthData(buffer);
      return VDS::Distances(buffer.response.buffer.data(), interface.resolution[0], interface.resolution[1]);
    }
  };

  VDS::VisionDepthSensor(std::string name, int index) throw(Exception)
    : Object(name, index),
      mImplPtr(new VDSImpl(*this)),
      resolution(mImplPtr->resolution())
  {}

  const VDS::Distances VDS::distances() throw(Exception) {
    return mImplPtr->distances();
  }
}
