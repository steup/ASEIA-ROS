#include "Attributes.h"

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <Transformation.h>

#include <MetaEvent.h>

#include <iostream>

namespace aseia_car_sim {

  using namespace std;
  using namespace ::id::attribute;

  class UTMToRoadTransformer : public Transformer {
    private:
      uint32_t mInRef, mOutRef;
      MetaEvent mRef;

      /** \todo implement **/
      MetaValue nurbsCoord(MetaValue in, const MetaValue& refIn,
                           const MetaValue& refOut, const MetaValue& nurbs) {
        ROS_ERROR_STREAM("UTM to Road Transform does not yet work");
        return in;
      }

    public:
      UTMToRoadTransformer(const EventType& out, const EventTypes& in)
        : Transformer(out, in) {
        mOutRef = out.attribute(Position::value())->scale().reference();
        mInRef = in[0].attribute(Position::value())->scale().reference();
      }

      virtual bool check(const MetaEvent& event) const {
        const MetaAttribute* attr = event.attribute(Position::value());
        if(!attr)
          return false;
        if(attr->scale().reference() == mInRef)
          return true;
        if(attr->scale().reference() == mOutRef)
          return true;
        return false;
      }

      virtual Events operator()(const MetaEvent& event) {
        ROS_DEBUG_STREAM("Executing Road to UTM Transform on " << event);
        const MetaAttribute* attr = event.attribute(Position::value());
        if(!attr)
          return {};
        if(attr->scale().reference() == mOutRef) {
          mRef = event;
          return {};
        }
        if(attr->scale().reference() == mInRef && mRef != MetaEvent()) {
          MetaEvent e = event;
          e.attribute(Position::value())->scale().reference(mOutRef);
          MetaValue& in     = e.attribute(Position::value())->value();
          const MetaValue& refIn  = mRef.attribute(Reference::value())->value();
          const MetaValue& refOut = mRef.attribute(Position::value())->value();
          const MetaValue& nurbs  = mRef.attribute(Nurbs::value())->value();
          in = nurbsCoord( in, refIn, refOut, nurbs);
          return {e};
        }
        return {};
      }

      virtual  void print(ostream& o) const {
        o << "UTM("  << mInRef << ") to Road(" << mOutRef << ") Transformer";
      }
  };

  class UTMToRoadTransformation : public Transformation {
    public:
      UTMToRoadTransformation() : Transformation(Transformation::Type::attribute, 2, EventID::any) {}

      virtual size_t arity() const { return 2; }

      virtual EventIDs in(EventID goal) const {
        EventID ref({Position::value(), Time::value(), Reference::value(), Nurbs::value()});
        return EventIDs({goal, ref});
      };

      virtual bool check(const EventType& out, const EventTypes& in) const {
        if(in.size()<2)
          return false;
        uint32_t outRef    = out.attribute(Position::value())->scale().reference();
        uint32_t inRef     = in[0].attribute(Position::value())->scale().reference();
        uint32_t refOutRef = in[1].attribute(Position::value())->scale().reference();
        uint32_t refInRef  = in[1].attribute(Reference::value())->scale().reference();
        return outRef == refOutRef && inRef == refInRef && inRef != outRef;
      }

      virtual TransPtr create(const EventType& out, const EventTypes& in, const AbstractPolicy& policy) const {
        return TransPtr(new UTMToRoadTransformer(out, in));
      }

      virtual void print(ostream& o) const {
        o << "UTM to Road Transformation";
      }
  };

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::UTMToRoadTransformation, Transformation)
