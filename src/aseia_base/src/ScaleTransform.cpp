#include <Transformation.h>

#include <iostream>
#include <MetaEvent>
#include <EventType>

class ScaleTransform : public Transformation {
  private:
    using Storage = std::map<id::attribute::ID, MetaScale>;
    Storage mScaleDeltas;

  protected:
    ScaleTransform(const EventType& out, const EventTypes& in) {
      if(in.size()!=1)
        return;
      const EventType& b = in[0];
      for(const AttributeType& aT : out) {
        AttributeType* temp = b.attribute(aT.id());
        if(!temp || b.scale() == temp->scale())
          continue;
        else
          mScaleDeltas.emplace(std::piecewise_construct, aT.id(),
            aT.scale()/temp->scale());
      }
    }
  public:
    virtual bool typeCheck(const EventType& out, const EventTypes& in) const {
      if(in.size()!=1 || !in[0])
        return false;
      const EventType& b = *in[0];
      if(EventID(out)!=EventID(b))
        return false;
      //TODO: check data types
      return true;
    }

    virtual bool check(const Events& events) const {
      return true;
    }

    virtual MetaEvent operator()(const Events& events) {
      if (events.size() != 1)
        return MetaEvent;
      MetaEvent e = events[0];
      for(MetaAttribute a : e) {
        const auto& it = mScaleDeltas.find(a.id());
        if(it != mScaleDeltas.end())
          a+=it->second;
      }
    }

    virtual print(std::ostream& o) const {
      o << "ScaleTransform: \n";
      for(const auto& subTrans : mScaleDeltas)
        o << "\t" << id::attribute::name(subTrans.first) << ": " << subTrans.second() << "\n";
      return o;
    }
};
