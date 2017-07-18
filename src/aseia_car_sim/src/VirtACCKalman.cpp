#include <pluginlib/class_list_macros.h>
#include <Transformation.h>
#include <ID.h>
#include <BaseEvent.h>

namespace aseia_car_sim {

using namespace std;
using namespace ::id::attribute;
class VirtACCKalmanTransformer : public Transformer {
  public:
    VirtACCKalmanTransformer(const EventType& goal, const EventTypes& in)
      : Transformer(goal, in) {

    }

    virtual bool check(const MetaEvent& e) const {
      // todo check for compatibility of speed to state or of future of sensing to state
      return false;
    }

    virtual Events operator()(const MetaEvent& e) {
      //todo handle speed event or dist event / speed -> control, dist -> sensing
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
    VirtACCKalman() : Transformation(Transformation::Type::homogeneus, 2, sDistID) {

    }

    virtual EventIDs in(EventID goal) const {
      if(goal == sDistID)
        return {sDistID, sSpeedID};
      else
        return {};
    }

    virtual EventTypes in(const EventType& goal, const EventType& provided) const {
      if(EventID(goal) == sDistID) {
        //todo define speed to use type of dist and position
        return {goal};
      }else
        return {};
    }

    virtual TransPtr create(const EventType& goal, const EventTypes& in, const AbstractPolicy& policy) const {
      return TransPtr(new VirtACCKalmanTransformer(goal, in));
    }

    virtual void print(ostream& o) const {
      o << "Kalman Transformation";
    }
};

const EventID VirtACCKalman::sDistID=EventID(BaseEvent<>())*Distance::value();
const EventID VirtACCKalman::sSpeedID=EventID(BaseEvent<>())*Speed::value();
}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::VirtACCKalman, Transformation)
