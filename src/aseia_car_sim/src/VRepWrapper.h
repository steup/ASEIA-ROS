#pragma once

#include <array>
#include <string>
#include <stdexcept>

#include <Eigen/Core>

namespace vrep {

  class Exception : public std::runtime_error {
    public:
    enum  class Reason {
      vrepGone,
      badResult,
      serviceGone
    };
    const Reason reason;
    private:
      static const char* genMsg(Reason) throw();
    public:
      Exception(Reason reason) throw();
  };

  class Object{
    private:
      Object() {}
    public:
      using Position = Eigen::Vector3f;
      using Orientation = Eigen::Matrix<float, 3, 3>;
      const int handle = -1;
      static const Object world;
      Object(std::string name, int index) throw(Exception);
      Position position(const Object& reference = world) const throw(Exception);
      Orientation orientation() const throw(Exception);
      std::string name() const throw(Exception);
  };

  class AngularJoint : public Object {
    private:
      float mAngle;
    public:
      AngularJoint(std::string name, int index) throw(Exception);
      float angle() const { return mAngle; }
      void angle(float angle) throw(Exception);
  };

  class VelocityJoint : public Object {
    private:
      float mVelocity;
    public:
      VelocityJoint(std::string name, int index) throw(Exception);
      float velocity() const { return mVelocity; }
      void velocity(float velocity) throw(Exception);
  };

  class ProximitySensor : public Object {
    public:
      ProximitySensor(std::string name, int index) throw(Exception);
      float distance() const throw(Exception);
  };

  class VDSImpl;

  class VisionDepthSensor : public Object {
    private:
      VDSImpl* mImplPtr;
    public:
      using Resolution = Eigen::Map<const Eigen::Array<int, 2, 1>>;
      using Distances = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>;
      const Resolution resolution;
      VisionDepthSensor(std::string name, int index) throw(Exception);
      const Distances distances() throw(Exception);
  };
}
