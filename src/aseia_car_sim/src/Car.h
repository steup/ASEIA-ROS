#pragma once

#include <cstdint>
#include <memory>
#include <map>
#include <vector>
#include <string>

namespace car {

  class Data {
    public:
      enum class Type {
        invalid,     //<< no such data
        in = 1,
        position,    //<< GPS
        lane,        //<< lane on road (-2, -1, | 1, 2, 3 )
        road,        //<< road ID
        offset,      //<< offset on road from start >0
        roadScan,    //<< 256 depth data of road
        licensePlate,//<< 16 char license plate
        out = 100,
        command,       //<< rad and km/h
      };
    private:
      const Type mType = Type::invalid;
      const std::uint64_t mTime = 0;
    public:
      Data() = default;
      Data(Type type, uint64_t time) : mType(type), mTime(time) {}
      Type type() const { return mType; }
      virtual void update() {}
      bool operator<(const Data& b) const { return mTime < b.mTime; }
  };

  using DataPtr       = std::unique_ptr< Data >;
  using DataVec       = std::vector< DataPtr >;
  using DataMap       = std::map< Data::Type, DataPtr >;
  using DataElem      = DataMap::value_type;

  class Controller {
    public:
      virtual void reference(const Data& ref) =0;
      virtual const Data& reference() const =0;
      virtual void operator()(DataMap& v) =0;
  };

  using ControllerPtr = std::unique_ptr< Controller >;

  ControllerPtr ctrlFactory(const std::string& name);

  class Car {
    private:
      DataMap mData;
      bool mAlive;
      using ControlVec = std::vector< ControllerPtr >;
      ControlVec mControl;
      const size_t mIndex;
      std::vector<std::string> mPubs;
    public:
      Car(std::size_t i);
      void addController(ControllerPtr control) { mControl.emplace_back(std::move(control)); }
      void update();
      Data speed() const;
      Data angle() const;
      bool alive() const { return mAlive; }
      std::string name() const;
      std::string frame() const;
  };
}
