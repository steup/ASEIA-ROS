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
    //MetaValue speed;
    const MetaValue object;//, object2;
    const MetaValue Q, z_alpha;
    VirtACCKalmanTransformer(const EventType& goal, const EventTypes& in, const MetaFilter& filter)
      : Transformer(goal, in), type(goal[Distance()].value().typeId()),
      object(extractValueFromFilter(Object(), filter)),// object2(extractValueFromFilter(Object2(), filter)),
      Q({{{1}}}, type), z_alpha({{{2.575829303549}}}, type) {
        x=MetaValue({{{0}}}, type); // distance
        P=MetaValue({{{1}}}, type); // sigma distance
        time=MetaValue({{{0}}}, goal[Time()].value().typeId());
      }

    virtual bool check(const MetaEvent& e) const {
      // todo check for compatibility of speed to state or of future of sensing to state
      return ( e.attribute(Distance()) && e[Object()].value() == object );// ||
//             ( e.attribute(Speed()) && e[Object()].value() == object);
    }

    virtual Events operator()(const MetaEvent& e) {
      /*if(e.attribute(Speed())) {
        speed = (e[Speed()]/e[Speed()].scale()).value(); // set speed
        return {};
      }
      if(speed.rows()==1) return {};*/
      MetaValue current = (e[Time()]/e[Time()].scale()).value();
      MetaValue dist = (e[Distance()]/e[Distance()].scale()).value();
      //MetaValue current = (e[Time()]/e[Time()].scale()).value(); // set new time
      MetaValue z = dist.value(); // sensor input (distance local or distance remote)
      MetaValue R = (dist.uncertainty()/z_alpha); // sensor input sigma
      R*=R; //sensor input covariance
      MetaValue Q_time = (current-time).value()*Q;

      MetaValue P_pre = P+Q_time; // prediction covariance
      MetaValue K = P_pre*(P_pre+R).inverse(); // kalman gain
      MetaValue x_temp = x + K*(z-x); // corrected state
      MetaValue P_temp = P_pre - K*P_pre; // corrected covariance
      //if time < current
      MetaEvent out=e;
      // convert covariance to uncertainty
      MetaValue result = P_temp.toUncertainty().sqrt()*z_alpha;
      result+=x;
      out[Distance()].value() = result; //set new output
      ROS_DEBUG_STREAM_NAMED("virt_acc_kalman", "Result: " << result << endl << "x: " << x_temp << endl << "P: " << P_temp << endl << "R: " << R << endl << "Q: " << Q_time << endl << "K: " << K << endl << "z: " << z << endl<< "out: " << out);
      if(x_temp.valid() && P_temp.valid()) {
        time == move(current);
        x = move(x_temp);
        P = move(P_temp);
      }

      return {out};

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
      return {sDistID};
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
      return {goal};
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
