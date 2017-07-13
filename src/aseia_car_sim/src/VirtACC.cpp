#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <Transformation.h>

#include <MetaEvent.h>
#include <IO.h>

namespace aseia_car_sim {

  using namespace std;
  using namespace ::id::attribute;

  class VirtACCTransformer : public BufferedTransformer {
    public:
      VirtACCTransformer(const EventType& out, const EventTypes& in)
        : BufferedTransformer(out, in, AbstractPolicy()) {
      }

      virtual bool check(const MetaEvent& event) const {
        return true;
      }

      virtual Events execute(const EventPtrs& inputs) const {
        const MetaAttribute& oID0 = *inputs[0]->attribute(Object::value());
        const MetaAttribute& oID1 = *inputs[1]->attribute(Object::value());
        const MetaAttribute& pos0 = *inputs[0]->attribute(Position::value());
        const MetaAttribute& pos1 = *inputs[1]->attribute(Position::value());
        const MetaAttribute& time0 = *inputs[0]->attribute(Time::value());
        const MetaAttribute& time1 = *inputs[1]->attribute(Time::value());
        if(oID0 == oID1 /*|| time0 != time1*/)
          return {};
        MetaEvent e0(out()), e1(out());
        e0=*inputs[0];
        e1=*inputs[1];
        MetaValue dist = (pos0.value()-pos1.value()).norm();
        e0.attribute(Distance::value())->value()=dist;
        e1.attribute(Distance::value())->value()=dist;
        return {e0, e1};
      }

      virtual  void print(ostream& o) const {
        o << "Virtual ACC Transformer";
      }
  };

  class VirtACC : public Transformation {
    private:
      static EventID mGoal;
    public:
      VirtACC() : Transformation(Transformation::Type::heterogeneus, 2, mGoal) {
        ROS_INFO_STREAM("VirtACC Transformation with goal id: " << mGoal);
      }

      virtual EventIDs in(EventID goal) const {
        ROS_INFO_STREAM("Testing VirtACC against goal: " << goal);
        if(goal == mGoal) {
          ROS_INFO_STREAM("VirtACC fits");
          return {goal/=Distance::value(), goal/=Distance::value()};
        }else
          return {};
      };

      virtual vector<EventType> in(const EventType& goal, const EventType& provided)  const {
        ROS_INFO_STREAM("Testing VirtACC against goal: " << goal);
        if(EventID(goal) != mGoal)
          return {};

        EventType in = goal;
        in.remove(Distance::value());

        ROS_INFO_STREAM("VirtACC fits with in type: " << in);
        return {in, in};
      }

      virtual TransPtr create(const EventType& out, const EventTypes& in, const AbstractPolicy& policy) const {
        return TransPtr(new VirtACCTransformer(out, in));
      }

      virtual void print(ostream& o) const {
        o << "Virtual ACC Transformation";
      }
  };

EventID VirtACC::mGoal({Position::value(), Distance::value(), Time::value(),
                                     Object::value(), PublisherID::value()});

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::VirtACC, Transformation)
