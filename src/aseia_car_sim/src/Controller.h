#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <ostream>

namespace car {

  class Car;

  class Controller {
    public:
      using Type = std::string;
    private:
      const Type mType;
      const std::size_t mPrio = 0;
    protected:
      Car& mCar;
    public:
      Controller(const Type& type, Car& car, const std::size_t prio)
        : mType(type), mPrio(prio), mCar(car)   {}
      virtual ~Controller() {}
      const Type& type() const { return mType; }
      virtual bool operator()() =0;
      virtual void print(std::ostream& o) const =0;
      bool operator<(const Controller& c) const { return mPrio < c.mPrio; }
  };

  inline std::ostream& operator<<(std::ostream& o, const Controller& c) { c.print(o); return o; }

  using ControllerPtr = std::unique_ptr< Controller >;

  ControllerPtr createController(const std::string& path, Car& car);
}
