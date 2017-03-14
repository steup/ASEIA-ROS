#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <Transformation.h>

#include <MetaEvent.h>

#include <iostream>

namespace aseia_car_sim {

  using namespace std;
  using namespace ::id::attribute;

  class RoadToUTMTransformer : public Transformer {
    private:
      uint32_t mInRef, mOutRef;
    public:
      RoadToUTMTransformer(const Transformation* t, const EventType& out, const EventTypes& in)
        : Transformer(t, out, in) {
        mOutRef = out.attribute(Position::value())->scale().reference();
        mInRef = in[0]->attribute(Position::value())->scale().reference();
      }

      virtual bool check(const Events& events) const {
        return true;
      }

      virtual MetaEvent operator()(const Events& events) {
        ROS_INFO_STREAM("Executing Road to UTM Transform");
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
        EventID ref({Position::value(), Time::value()});
        return EventIDs({goal, ref});
      };

      virtual bool check(const EventType& out, const EventTypes& in) const {
        bool fits = true;
        fits |= out.attribute(Position::value())->scale().reference()==1;
        fits |= in[0]->attribute(Position::value())->scale().reference()==0;
        return fits;
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
