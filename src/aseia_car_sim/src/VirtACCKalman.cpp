#include <pluginlib/class_list_macros.h>
#include <Transformation.h>
#include <ID.h>
#include <BaseEvent.h>
#include <IO.h>

#include <ros/ros.h>

namespace aseia_car_sim {

using namespace std;
using namespace ::id::attribute;
class VirtACCKalmanTransformer : public Transformer {
  public:
    VirtACCKalmanTransformer(const EventType& goal, const EventTypes& in, const MetaFilter& filter)
      : Transformer(goal, in) {
    }

    virtual bool check(const MetaEvent& e) const {
      // todo check for compatibility of speed to state or of future of sensing to state
      return true;
    }

    virtual Events operator()(const MetaEvent& e) {
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
    static const EventID sPositionID;
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
      return {sPositionID, sDistID, sSpeedID};
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
      //todo define speed to use type of dist and position
      return {position, speed, goal};
    }

    virtual TransPtr create(const EventType& goal, const EventTypes& in, const AbstractPolicy& policy, const MetaFilter& filter= MetaFilter()) const {
      return TransPtr(new VirtACCKalmanTransformer(goal, in, filter));
    }

    virtual void print(ostream& o) const {
      o << "Kalman Transformation";
    }
};
const EventID VirtACCKalman::sPositionID=EventID(BaseEvent<>());
const EventID VirtACCKalman::sDistID=EventID(BaseEvent<>())*Distance::value();
const EventID VirtACCKalman::sSpeedID=EventID(BaseEvent<>())*Speed::value();
}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::VirtACCKalman, Transformation)
