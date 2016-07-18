#pragma once

#include <Transformation>
#include <stdint.h>

class TransformationRegistryImpl;

class TransformationFactory {
  public:
    TransID = std::uint32_t;
  private:
    using EventTypes = Transformation::EventTypes;
    using createFunc = TransPtr(*)(const EventType& out, const EventTypes& in);
    static TransformationRegistryImpl* mImpl;
    virtual TransID registerCreator(createFunc) =0

  public:
    static TransformationFactory instance() { return mImpl; }
    virtual TransPtr create(const EventType&out, const EventTypes& in, TransID trans) const =0;

    template<class T>
    TransID registerTransformation() {
    createFunc f = &Transformation::create<T>;
      registerCreator(createFunc);
    }
};
