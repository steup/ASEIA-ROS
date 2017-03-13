#include <pluginlib/class_list_macros.h>
#include <Transformation.h>

#include <MetaEvent.h>

#include <iostream>

namespace aseia_car_sim {

  using namespace std;

  class RoadToUTMTransformer : public Transformer {
    private:
      uint32_t mInRef, mOutRef;
    public:
      RoadToUTMTransformer(const Transformation* t, const EventType& out, const EventTypes& in)
        : Transformer(t, out, in) {
        //TODO Extract Scale Reference IDs
      }

      virtual bool check(const Events& events) const {
        return true;
      }

      virtual MetaEvent operator()(const Events& events) {
        return MetaEvent();
      }

      virtual  void print(ostream& o) const {
        o << "Road("  << mInRef << ") to UTM(" << mOutRef << ") Transformer";
      }
  };

  class RoadToUTMTransformation : public Transformation {
    public:
      RoadToUTMTransformation() : Transformation(EventID::any) {}

      virtual size_t arity() const { return 2; }

      virtual EventIDs in(EventID goal) const { 
        //TODO Create Reference Event
        return EventIDs({goal}); 
      };

      virtual bool check(const EventType& out, const EventTypes& in) const { 
        return false; 
      }

      virtual TransPtr create(const EventType& out, const EventTypes& in) const { 
        return TransPtr(new RoadToUTMTransformer(this, out, in)); 
      }

      virtual void print(ostream& o) const { 
        o << "not yet implemented road to UTM transformation"; 
      }
  };

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::RoadToUTMTransformation, Transformation)
