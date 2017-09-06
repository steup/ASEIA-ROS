#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <Transformation.h>

#include <MetaEvent.h>
#include <IO.h>


#include <map>

namespace aseia_car_sim {

  static const char* transName = "virt_acc";

  using namespace std;
  using namespace ::id::attribute;

  class VirtACCTransformer : public Transformer {
    private:
      using Storage = vector<MetaEvent>;
      Storage mStorage;
    public:
      VirtACCTransformer(const EventType& out, const EventTypes& in)
        : Transformer(out, in) {
      }

      virtual bool check(const MetaEvent& event) const {
        return true;
      }

      virtual Events operator()(const MetaEvent& e) {

        const MetaAttribute& oID0 = *e.attribute(Object::value());

        auto it = find_if(mStorage.begin(), mStorage.end(), [oID0](const MetaEvent& e){ return *e.attribute(Object::value())==oID0; });
        if(it == mStorage.end())
          mStorage.push_back(e);
        else
          *it = e;

        Events events;

        for(const MetaEvent& v: mStorage) {
          const MetaAttribute& oID1 = *v.attribute(Object::value());

          if(oID0 == oID1)// || (time0 - time1).value().norm() < 100)
            continue;
          const MetaValue& ori = e[Orientation()].value();
          MetaValue diff = (v[Position()]-e[Position()]).value();
          if((ori*diff).sum() < 5) {
            ROS_DEBUG_STREAM_NAMED(transName, "Filtered pair: " << oID0 << ", " << oID1);
            continue;
          }
          ROS_DEBUG_STREAM_NAMED(transName, "ori: " << ori << endl << "diff: " << diff << endl << "dot: " << ori*diff);
          ROS_DEBUG_STREAM_NAMED(transName, "Producing Event for pair: " << oID0 << ", " << oID1);
          MetaEvent e0(out());
          e0=e;
          MetaAttribute offset(out()[Distance()]);
          offset/=offset.scale();
          offset.value()=offset.value().zero();
          offset.value().block(0,0, MetaValue(8, out()[Distance()].value().typeId()));
          offset*=MetaScale(out()[Distance()].scale());
          e0[Distance()]=(e0[Position()]-v[Position()]).norm()-offset;
          ROS_DEBUG_STREAM_NAMED(transName, "Insert Event in output queue: " << e0);
          if(e0[Distance()].value().value()<200)
            events.push_back(e0);
        }
        ROS_DEBUG_STREAM_NAMED(transName, "Outputting resulting events");
        for(const MetaEvent& res : events)
          ROS_DEBUG_STREAM_NAMED(transName, "Resulting Events:" << res);
        return events;
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
        ROS_DEBUG_STREAM_NAMED(transName, "VirtACC Transformation with goal id: " << mGoal);
      }

      virtual EventIDs in(EventID goal, const MetaFilter& = MetaFilter()) const {
        ROS_DEBUG_STREAM_NAMED(transName, "Testing VirtACC against goal: " << goal);
        if(goal == mGoal) {
          ROS_INFO_STREAM("VirtACC fits");
          return {goal/=Distance::value(), goal/=Distance::value()};
        }else
          return {};
      };

      virtual vector<EventType> in(const EventType& goal, const EventType& provided, const MetaFilter& = MetaFilter())  const {
        ROS_DEBUG_STREAM_NAMED(transName, "Testing VirtACC against goal: " << goal);
        if(EventID(goal) != mGoal)
          return {};

        EventType in = goal;
        in.remove(Distance::value());

        ROS_DEBUG_STREAM_NAMED(transName, "VirtACC fits with in type: " << in);
        return {in, in};
      }

      virtual TransPtr create(const EventType& out, const EventTypes& in, const AbstractPolicy& policy, const MetaFilter& = MetaFilter()) const {
        return TransPtr(new VirtACCTransformer(out, in));
      }

      virtual void print(ostream& o) const {
        o << "Virtual ACC Transformation";
      }
  };

EventID VirtACC::mGoal({Position(), Distance(), Time(), Object::value(), Orientation(), PublisherID()});

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::VirtACC, Transformation)
