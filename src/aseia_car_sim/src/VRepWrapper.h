#pragma once

#include <array>
#include <string>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vrep {

  class Exception : public std::exception {
    public:
    enum  class Reason {
      vrepGone,
      badResult,
      serviceGone
    };
    const Reason reason;
    public:
      Exception(Reason reason) throw() : reason(reason) {}
      virtual const char* what() const throw();

  };

  class ExtendedException : public Exception {
    private:
      std::ostringstream mOs;
    public:
      ExtendedException(const Exception& e) : Exception(e) {}
      ExtendedException(Reason reason) : Exception(reason) {}
      ExtendedException(const ExtendedException& e) : Exception(e) { mOs << e.mOs.str(); }
      //ExtendedException(ExtendedException&& e) : Exception(e), mOs(std::move(e.mOs)) {}
      virtual const char* what() const throw();
      template<typename T>
      ExtendedException& operator<<(const T& t){ mOs << t; return *this;}
  };

  class Object{
    private:
      Object() {}
    public:
      using Position = Eigen::Vector3f;
      using Orientation = Eigen::Quaternionf;
      using Speed = float;
      const int handle = -1;
      static const Object world;
      Object(std::string name, int index) throw(Exception);
      Position position(const Object& reference = world) const throw(Exception);
      Orientation orientation(const Object& reference = world) const throw(Exception);
      Speed speed() const throw(Exception);
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
  
  uint32_t getTime();
}
