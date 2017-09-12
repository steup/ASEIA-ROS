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
    MetaFilter adaptFilter(const MetaFilter& filter) {
      MetaFilter temp(filter);
      auto pred=[](const pair<MetaPredicate, filterOp::ID>& subExpr){
        const MetaPredicate& p=subExpr.first;
        return p.event() !=0 ||
               ( p.attribute() == Distance() &&
                ( p.op() == filterOp::LE() || p.op() == filterOp::LT() ));
      };
      auto& expr =  temp.expressions();
      expr.erase(remove_if(expr.begin(), expr.end(), pred), expr.end());
      return temp;
    }
  public:
    const type::ID type;
    MetaAttribute x, P, time, Q, z_alpha, limit;
    VirtACCKalmanTransformer(const EventType& goal, const EventTypes& in, const MetaFilter& filter)
      : Transformer(goal, in),
        type(goal[Distance()].value().typeId()),
        x(goal[Distance()]),
        P(goal[Distance()]),
        time(goal[Time()]),
        Q(goal[Distance()]),
        z_alpha(goal[Distance()]),
        limit(goal[Distance()])
      {
        mFilter = adaptFilter(filter);
        x.value()=MetaValue(type, 1, 1, false).zero();
        P.value()=MetaValue(type, 1, 1, false).ones();
        P.unit()*=P.unit();
        time.value()=MetaValue(goal[Time()].value().typeId(), 1, 1, false).zero();
        Q.value() = MetaValue({{{0.1}}}, type);
        Q.unit() = Meter()*Meter()/Second();
        z_alpha.value() = MetaValue({{{2.575829303549}}}, type);
        z_alpha.unit() = Dimensionless();
        limit.value() = MetaValue({{{100, 0}}},  type);
      }

    virtual bool check(const MetaEvent& e) const {
      // todo check for compatibility of speed to state or of future of sensing to state
      return mFilter({&e});// ||
//             ( e.attribute(Speed()) && e[Object()].value() == object);
    }

    virtual Events operator()(const MetaEvent& e) {
      if(e[Time()] < time || !mFilter({&e}) )
          return {};
      MetaAttribute z = e[Distance()].valueOnly();
      MetaAttribute R = e[Distance()].uncertainty()/z_alpha; // sensor input sigma
      R*=R; //sensor input covariance
      MetaAttribute Q_time = (e[Time()].valueOnly()-time).norm()*Q;

      MetaAttribute P_pre = P+Q_time; // prediction covariance
      MetaAttribute K = P_pre*(P_pre+R).inverse(); // kalman gain
      MetaAttribute x_temp = x + K*(z-x); // corrected state
      MetaAttribute P_temp = P_pre - K*P_pre; // corrected covariance
      //if time < current
      MetaEvent out=e;
      // convert covariance to uncertainty
      MetaAttribute result = P_temp.toUncertainty().sqrt()*z_alpha;
      ROS_DEBUG_STREAM_NAMED("virt_acc_kalman", "x_temp:" << x_temp << endl << "Result: " << result << endl << "P_temp" << P_temp << endl << "x: " << x_temp << endl << "P: " << P_temp << endl << "R: " << R << endl << "Q: " << Q_time << endl << "K: " << K << endl << "z: " << z << endl<< "out: " << out << endl << "diff: " << (x-result));
      result+=x_temp;
      out[Distance()] = result; //set new output
      if(x_temp.valid() && P_temp.valid() && x_temp < limit) {
        time = e[Time()].valueOnly();
        x = move(x_temp);
        P = move(P_temp);
        return {out};
      }
      else
        P=move(P_pre);

      return {};

    }

    virtual void print(ostream& o) const {
      o << "Virtual ACC Sensor Kalman Transformer with filter " << mFilter;
    }
};

class VirtACCKalman : public Transformation {
  protected:
    static const EventID sDistID;
  public:
    VirtACCKalman() : Transformation(Transformation::Type::homogeneus, 3, EventID::any) {

    }
    bool containsUncertaintyTest(const MetaFilter& filter) const {
      for(const auto& predicate : filter.expressions())
        for(const auto& func : predicate.first.func())
          if(func==&MetaAttribute::uncertainty)
            return true;
      return false;
    }

    virtual EventIDs in(EventID goal, const MetaFilter& filter = MetaFilter()) const {
      if(!containsUncertaintyTest(filter) || !goal.isCompatible(sDistID)) return {};
      ROS_DEBUG_STREAM_NAMED("virt_kalman", "Kalman Input IDs for id " << goal);
      return {goal};
    }

    virtual EventTypes in(const EventType& goal, const EventType& provided, const MetaFilter& filter = MetaFilter()) const {
      if(!containsUncertaintyTest(filter)) return {};
      ROS_DEBUG_STREAM_NAMED("virt_kalman", "Kalman Input Types for type " << goal);
      return {goal};
    }

    virtual TransPtr create(const EventType& goal, const EventTypes& in, const AbstractPolicy& policy, const MetaFilter& filter= MetaFilter()) const {
      return TransPtr(new VirtACCKalmanTransformer(goal, in, filter));
    }

    virtual void print(ostream& o) const {
      o << "Kalman Transformation";
    }
};

const EventID VirtACCKalman::sDistID=EventID(BaseEvent<>())*Distance()*Orientation();

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::VirtACCKalman, Transformation)
