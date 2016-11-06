#pragma once

#include <string>

namespace car {

  class Data;

  class Car {
    protected:
        bool         mAlive = true;
        unsigned int mIndex;
    public:
      Car(std::size_t i) : mIndex(i) {}
      virtual ~Car() {}
      bool alive() const { return mAlive; }
      int index() const { return mIndex; }
      virtual const Data* getReference(const std::string& name) const =0;
      virtual const Data* getSensor   (const std::string& name) const =0;
      virtual       Data* getActuator (const std::string& name) =0;
  };
}
