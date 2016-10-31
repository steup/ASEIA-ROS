#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <ostream>
#include <unordered_map>

namespace car {

  class Data {
    public:
      using Type = std::string;
    private:
      const std::string mType;
      const bool mInput, mOutput;
    public:
      Data(const Type type, bool input=false, bool output=false) : mType(type), mInput(input), mOutput(output) {};
      virtual ~Data() {}
      const std::string& type() const { return mType; }
      bool isInput() const { return mInput; }
      bool isOutput() const { return mOutput; }
      virtual void update() {};
      virtual void print(std::ostream& o) const { o << "unknown data"; };
  };

  inline std::ostream& operator<<(std::ostream& o, const Data& d) { d.print(o); return o; }

  using DataPtr       = std::unique_ptr< Data >;
  using DataMap       = std::unordered_map< std::string, DataPtr >;

  class Controller {
    public:
      using Type = std::string;
    private:
      const Type mType;
      const std::size_t mPrio;
    public:
      Controller(Type type, size_t prio) : mType(type), mPrio(prio) {}
      virtual ~Controller() {}
      const Type& type() const { return mType; }
      virtual void operator()(DataMap& v) =0;
      virtual void print(std::ostream& o) const =0;
      bool operator<(const Controller& c) const { return mPrio < c.mPrio; }
  };

  inline std::ostream& operator<<(std::ostream& o, const Controller& c) { c.print(o); return o; }

  using ControllerPtr = std::unique_ptr< Controller >;
}
