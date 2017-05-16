#include "Attributes.h"

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <Transformation.h>

#include <MetaEvent.h>
#include <IO.h>

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

  class UTMToRoad : public Transformation {
    public:
      UTMToRoad() : Transformation(Transformation::Type::attribute, 2, EventID::any) {}

      virtual EventIDs in(EventID goal) const {
        EventID ref({Position::value(), Time::value(), Orientation::value(), Reference::value(), Nurbs::value()});
        ROS_INFO_STREAM("Querying UTMToRoadTransformation for appropriate Input EventIDs: [" << goal << ", " << ref << "]");
        return EventIDs({goal, ref});
      };

      virtual vector<EventType> in(const EventType& goal, const EventType& provided)  const {
        if(provided==EventType())
          return {};
        const AttributeType* goalTimeAT = goal.attribute(Time::value());
        const AttributeType* goalPosAT = goal.attribute(Position::value());
        const AttributeType* providedPosAT = provided.attribute(Position::value());
        if(!goalPosAT || !goalTimeAT || !providedPosAT
           || goalPosAT->scale().reference() == providedPosAT->scale().reference())
          return {};
        uint32_t oriDim, nurbsDim=100;
        ::id::type::ID type = ValueType(goalPosAT->value()).typeId();
        if(goalPosAT->value().rows() == 3)
          oriDim = 3;
        else
          oriDim = 1;

        EventType orig = goal;
        orig.attribute(Position::value())->scale().reference(providedPosAT->scale().reference());
        //todo handle possible orientation
        EventType reference;
        reference.add(AttributeType(Reference::value(), providedPosAT->value(), providedPosAT->scale(), providedPosAT->unit()));
        reference.add(AttributeType(Orientation::value(), ValueType(type, oriDim, 1, true), providedPosAT->scale(), Radian()));
        reference.add(*goalTimeAT);
        reference.add(AttributeType(Nurbs::value(), ValueType(type, nurbsDim, 1, false), goalPosAT->scale(), providedPosAT->unit()));

        auto input = {orig, reference};
        ROS_INFO_STREAM("UTMToRoad [" << goal << ", " << provided << "] -> [" << orig << ", " << reference << "]");
        return input;
      }

      virtual TransPtr create(const EventType& out, const EventTypes& in, const AbstractPolicy& policy) const {
        return TransPtr(new UTMToRoadTransformer(out, in));
      }

      virtual void print(ostream& o) const {
        o << "UTM to Road Transformation";
      }
  };

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::UTMToRoad, Transformation)
