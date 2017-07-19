#include <pluginlib/class_list_macros.h>
#include <Transformation.h>
#include <ID.h>
#include <BaseEvent.h>

namespace aseia_car_sim {

using namespace std;
using namespace ::id::attribute;

class VirtSpeedTransformer : public Transformer {
  private:
      using Storage = vector<MetaEvent>;
      Storage mStorage;
  public:
    VirtSpeedTransformer(const EventType& goal, const EventTypes& in)
      : Transformer(goal, in) {

    }

    virtual bool check(const MetaEvent& e) const {
      return true;
    }

    virtual Events operator()(const MetaEvent& e0) {
      const MetaAttribute& oID0 = e0[Object()];
      auto it = find_if(mStorage.begin(), mStorage.end(),
                        [&oID0](const MetaEvent& e){
                          return e[Object()]==oID0;
                        });
      if(it == mStorage.end()) {
        mStorage.push_back(e0);
        return {};
      }

      MetaEvent& e1 = *it;

      if(e0[Time()] >= e1[Time()] ||
        (e0[Time()] - e1[Time()]) < 1 ) {
        return {};
      }

      MetaEvent eOut(out());
      eOut=(e0+e1)/2;
      eOut[Speed()]=(e0[Position()]-e1[Position()]).norm()/(e0[Time()]-e1[Time()]);
      e1 = e0;
      return {eOut};
    }

    virtual void print(ostream& o) const {
      o << "Speed Transformer";
    }
};

class VirtSpeed : public Transformation {
  protected:
    static const EventID sSpeedID;
  public:
    VirtSpeed() : Transformation(Transformation::Type::heterogeneus, 2, sSpeedID) {

    }

    virtual EventIDs in(EventID goal) const {
      if(goal == sSpeedID)
        return {sSpeedID/Speed(), sSpeedID/Speed()};
      else
        return {};
    }

    virtual EventTypes in(const EventType& goal, const EventType& provided) const {
      if(EventID(goal) == sSpeedID) {
        EventType temp = goal;
        temp.remove(Speed());
        return {temp, temp};
      }else
        return {};
    }

    virtual TransPtr create(const EventType& goal, const EventTypes& in, const AbstractPolicy& policy) const {
      return TransPtr(new VirtSpeedTransformer(goal, in));
    }

    virtual void print(ostream& o) const {
      o << "Speed Transformation";
    }
};

const EventID VirtSpeed::sSpeedID=EventID(BaseEvent<>())*Object()*Speed();
}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::VirtSpeed, Transformation)