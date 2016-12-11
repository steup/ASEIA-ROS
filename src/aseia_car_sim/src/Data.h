#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <ostream>
#include <unordered_map>

#include <Eigen/Core>

namespace car {

  class Car;

  class Data {
    public:
      using Type = std::string;
    private:
      const Type mType;
      const bool mInput, mOutput;
    protected:
      const Car& mCar;
    public:
      Data(const Type& type, const Car& car, bool input=false, bool output=false)
        : mType(type), mInput(input), mOutput(output), mCar(car){}
      virtual ~Data() {}
      const Type& type() const { return mType; }
      bool isInput() const { return mInput; }
      bool isOutput() const { return mOutput; }
      virtual bool update() =0;
      virtual void print(std::ostream& o) const =0;
  };

  inline std::ostream& operator<<(std::ostream& o, const Data& d) { d.print(o); return o; }

  class FloatArray  : public Data, public Eigen::Array<float, Eigen::Dynamic, 1>{
    private:
      using Base = Eigen::Array<float, Eigen::Dynamic, 1>;
    public:
      FloatArray(const std::string& path, const Car& car, size_t n, bool input=false, bool output=false)
        : Data("float"+std::to_string(n), car, input, output),
          Base(n)
      {}
      virtual void print(std::ostream& o) const { o << (const Base&)(*this); }
  };

  class Float : public Data {
    protected:
      float mValue = 0.0f;
    public:
      Float(const std::string& path, const Car& car, bool input = false, bool output = false)
        : Data("float", car, input, output)
      {}
      float value() const { return mValue; }
      void value(float v) { mValue = v; }
      virtual bool update() { return true; }
      virtual void print(std::ostream& o) const { o << type() << " " << mValue; }
  };

  using DataPtr       = std::unique_ptr< Data >;
  DataPtr createData(const std::string& path, Car& car);
}
