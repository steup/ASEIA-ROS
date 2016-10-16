#pragma once

#include <cstdint>
#include <memory>
#include <map>
#include <vector>

namespace car {

  class Data {
    public:
      enum class Type {
        invalid,     //<< no such data
        position,    //<< GPS
        speed,       //<< km/h
        angle,       //<< rad
        lane,        //<< lane on road (-2, -1, | 1, 2, 3 )
        road,        //<< road ID
        offset,      //<< offset on road from start >0
        roadScan,    //<< 256 depth data of road
        licensePlate,//<< 16 char license plate
      };
    private:
      const Type mType;
      const std::uin64_t time;
    public:
      Data(Type type) : mType(type) {}
      Type type() const { return mType; }
      bool operator<(const Data& b) const { return time < b.time; }
  };


  class Controller {
    public:
      virtual void reference(const Data& ref);
      virtual const Reference& reference(Data::Type type) const;
      virtual Commands operator()(const DataVec& v);
  };

  using DataPtr       = std::unique_ptr< Data >;
  using DataVec       = std::vector< DataPtr >
  using DataMap       = std::map< Data::Type, DataVec >;
  using ControllerPtr = std::unique_ptr< Controller >

  class Car {
    private:
      DataMap mData;
      bool mAlive;
      using ControlVec = std::vector< ControllerPtr >;
      ControlVec mControl;
    public:
      Car() : mAlive(true) {}
      void addController(ControllerPtr control);
      void feed(const Data& data);
      void update();
      const Data& speed() const;
      const Data& angle() const;
      bool alive() const { return mAlive; }
  };
}
