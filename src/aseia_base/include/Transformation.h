#pragma once

#include <iosfwd>
#include <list>
#include <memory>

class MetaEvent;

class Transformation {
  protected:
    Transformation() = default;

  public:
    using EventTypes = std::list<const EventType*>;
    using Events = std::list<const MetaEvents*>;
    using TransID = uint32_t;
    using TransPtr = std::unique_ptr<Transformation>;

    template<typename T>
    static create(const EventType& out, const EventTypes& in) {
      return TransPtr(new T(out, in));
    }

    virtual bool typeCheck(const EventType& out, const EventTypes& in) const =0;
    virtual bool check(const Events& events) const =0;
    virtual MetaEvent operator()(const Events& events) =0;
    virtual print(std::ostream& o) const = 0;
};
