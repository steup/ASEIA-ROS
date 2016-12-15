#pragma once

#include <memory>
#include <string>
#include <ostream>

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

  class Float : public Data {
    public:
      float value = 0.0f;
      Float(const std::string& path, const Car& car, bool input = false, bool output = false)
        : Data("float", car, input, output)
      {}
      virtual bool update() { return true; }
      virtual void print(std::ostream& o) const { o << type() << " " << value; }
      operator float() const { return value; }
  };

  using DataPtr       = std::unique_ptr< Data >;
  DataPtr createData(const std::string& path, Car& car);
}
