#include <pluginlib/class_list_macros.h>
#include <Transformation.h>
#include <ID.h>
#include <BaseEvent.h>
#include <IO.h>

#include <ros/ros.h>

namespace aseia_car_sim {

using namespace std;
using namespace ::id::attribute;
using namespace ::id;

class VirtACCKalmanTransformer : public Transformer {
  private:
    MetaValue extractValueFromFilter(ID attr, const MetaFilter& filter) {
      for(const auto& expr : filter.expressions()) {
        const MetaPredicate& p = expr.first;
        if(p.attribute()==attr && p.constArgument())
          return p.argument().value();
        }
      return MetaValue();
    }
  public:
    const type::ID type;
    MetaValue x, P;
    MetaValue time;
    MetaValue speed0, speed1;
    const MetaValue object, object2;
    const MetaValue F1, B1, H1, Q, R;
    VirtACCKalmanTransformer(const EventType& goal, const EventTypes& in, const MetaFilter& filter)
      : Transformer(goal, in), type(goal[Distance()].value().typeId()),
      object(extractValueFromFilter(Object(), filter)), object2(extractValueFromFilter(Object2(), filter)),
      F1({{{0}}}, type), B1({{{0}}}, type),
      H1({{{0}}}, type),
      Q({{{0}}}, type), R({{{0}}}, type) {
    }

    virtual bool check(const MetaEvent& e) const {
      // todo check for compatibility of speed to state or of future of sensing to state
      return ( e.attribute(Distance()) && e[Object()].value() == object && e[Object2()].value() == object2 ) ||
             ( e.attribute(Speed()) && ( e[Object()].value() == object || e[Object()].value() == object2 ) );
    }

    virtual Events operator()(const MetaEvent& e) {
      MetaValue z(type, 3, 1, true);
      z = z.zero();
      if(e.attribute(Distance())) {
        z.block(0,0, e[Distance()].value());
      }
      if(e.attribute(Speed())) {
        if(e[Object()].value()==object)
          z.block(1,0, e[Speed()].value());
        else
          z.block(2,0, e[Speed()].value());
      }
      ROS_DEBUG_STREAM_NAMED("virt_acc_kalman", "z: " << z);
      //todo handle speed event or dist event / speed -> control, dist -> sensing
      if(e.attribute(Distance()))
        return {e};
      else
        return {};

    }

    virtual void print(ostream& o) const {
      o << "Unimplemeneted Kalman Transformer";
    }
};

class VirtACCKalman : public Transformation {
  protected:
    static const EventID sDistID;
    static const EventID sSpeedID;
  public:
    VirtACCKalman() : Transformation(Transformation::Type::homogeneus, 3, sDistID) {

    }
    bool containsUncertaintyTest(const MetaFilter& filter) const {
      for(const auto& predicate : filter.expressions())
        for(const auto& func : predicate.first.func())
          if(func==&MetaAttribute::uncertainty)
            return true;
      return false;
    }

    virtual EventIDs in(EventID goal, const MetaFilter& filter = MetaFilter()) const {
      if(!containsUncertaintyTest(filter)) return {};
      ROS_DEBUG_STREAM_NAMED("virt_kalman", "Kalman Input IDs for id " << goal);
      return {sDistID, sSpeedID};
    }

    virtual EventTypes in(const EventType& goal, const EventType& provided, const MetaFilter& filter = MetaFilter()) const {
      if(!containsUncertaintyTest(filter)) return {};
      ROS_DEBUG_STREAM_NAMED("virt_kalman", "Kalman Input Types for type " << goal);
      EventType position = goal;
      position.remove(Distance());
      const AttributeType& distAttr = goal[Distance()];
      const AttributeType& timeAttr = goal[Time()];
      EventType speed = position;
      AttributeType speedAttr(Speed(), distAttr.value(), MetaScale(distAttr.scale())/timeAttr.scale(), MetaUnit(distAttr.unit())/timeAttr.unit());
      speed.add(speedAttr);
      return {speed, goal};
    }

    virtual TransPtr create(const EventType& goal, const EventTypes& in, const AbstractPolicy& policy, const MetaFilter& filter= MetaFilter()) const {
      return TransPtr(new VirtACCKalmanTransformer(goal, in, filter));
    }

    virtual void print(ostream& o) const {
      o << "Kalman Transformation";
    }
};

const EventID VirtACCKalman::sDistID=EventID(BaseEvent<>())*Distance::value();
const EventID VirtACCKalman::sSpeedID=EventID(BaseEvent<>())*Speed::value();

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::VirtACCKalman, Transformation)
