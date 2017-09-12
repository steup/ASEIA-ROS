#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <Transformation.h>

#include <MetaEvent.h>
#include <IO.h>

#include <map>
#include <vector>
#include <functional>

namespace aseia_car_sim {

  static const char* transName = "virt_acc";

  using namespace std;
  using namespace ::id::attribute;

  class VirtACCTransformer : public Transformer {
    private:
      struct comp {
        bool operator()(const MetaAttribute& a, const MetaAttribute& b) const {
          return (bool)(a<b);
        }
      };
      struct timeComp {
        bool operator()(const MetaEvent& a, const MetaEvent& b) const {
          return (bool)(a[Time()]>b[Time()]);
        }
      };

      using Storage = map<MetaAttribute, vector<MetaEvent>, comp>;
      Storage mStorage;
      MetaAttribute posRef;
      MetaAttribute timeRef;
      MetaAttribute distRef;
    public:
      VirtACCTransformer(const EventType& out, const EventTypes& in)
        : Transformer(out, in), posRef(in[0][Position()]), timeRef(in[0][Time()]), distRef(out[Distance()]) {
          posRef.value().set(0,0, {10});
          timeRef.value().set(0,0, {1});
          distRef.value().set(0,0, {200});
      }

      virtual bool check(const MetaEvent& event) const {
        MetaAttribute pos = event[Position()]/event[Position()].scale();
        MetaAttribute time= event[Time()]/event[Time()].scale();
        return pos.uncertainty() < posRef && time.uncertainty() < timeRef;
      }

      virtual Events operator()(const MetaEvent& e) {
        ostringstream os;
        const MetaAttribute& oID0 = *e.attribute(Object::value());
        auto oIDIt = mStorage.find(oID0);
        if(oIDIt == mStorage.end())
          oIDIt = mStorage.insert(make_pair(oID0, vector<MetaEvent>())).first;
        if(oIDIt->second.size() <10)
          oIDIt->second.push_back(e);
        else
          oIDIt->second.back() = e;
        sort(oIDIt->second.begin(), oIDIt->second.end(), timeComp());
        Events events;
        for(const MetaEvent& e: mStorage[oID0]) {
          bool found = false;
          for(const auto& p : mStorage) {
            if(p.first == oID0) {
              os << "Ignoring events of same ObjectID: " << oID0 << " == " << p.first << endl;
              continue;
            }
            for(const MetaEvent& v: p.second) {
              if(v[Time()] > e[Time()]) {
                os << v << endl << " is newer then " << e << ": discarding!" << endl;
                continue;
              }
              if(v[Time()] < e[Time()]) {
                os << v << endl << " is older then " << e << ": aborting search!" << endl;
                break;
              }

              const MetaAttribute& oID1 = p.first;
              const MetaValue& ori = e[Orientation()].value();
              MetaValue diff = (v[Position()]-e[Position()]).value();
              if((ori*diff).sum() < 5) {
                os << "Filtered pair: " << oID0 << ", " << v[Object()] << endl;
                found=true;
                break;
              }
              os << "ori: " << ori << endl << "diff: " << diff << endl << "dot: " << ori*diff << endl;

              os << "Producing Event for pair: " << oID0 << ", " << oID1 << endl;
              MetaEvent e0(out());
              e0=e;
              MetaAttribute offset(out()[Distance()]);
              offset/=offset.scale();
              offset.value()=MetaValue({{{4, 0}}}, out()[Distance()].value().typeId());
              offset*=MetaScale(out()[Distance()].scale());
              e0[Distance()]=(e0[Position()]-v[Position()]).norm()-offset;
              os << "Offset: " << offset << endl;
              os << "Insert Event in output queue: " << e0 << endl;
              if(e0[Distance()]/e0[Distance()].scale() < distRef) {
                events.push_back(e0);
                found=true;
                break;
              } else
                os << e0 << endl << " contains a distance larger then maximum of 200 m discarding!" << endl;
            }
          }
          if(found)
            break;
        }
        os << events.size() << " Events generated" << endl;
        ROS_DEBUG_STREAM_NAMED(transName, os.str());
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
      VirtACC() : Transformation(Transformation::Type::heterogeneus, 1, mGoal) {
        ROS_DEBUG_STREAM_NAMED(transName, "VirtACC Transformation with goal id: " << mGoal);
      }

      virtual EventIDs in(EventID goal, const MetaFilter& = MetaFilter()) const {
        ROS_DEBUG_STREAM_NAMED(transName, "Testing VirtACC against goal: " << goal);
        if(goal.isCompatible(mGoal))
          return {goal/Distance()*Orientation()};
        else
          return {};
      };

      virtual vector<EventType> in(const EventType& goal, const EventType& provided, const MetaFilter& = MetaFilter())  const {
        ROS_DEBUG_STREAM_NAMED(transName, "Testing VirtACC against goal: " << goal);
        EventType in = goal;
        in.remove(Distance());
        AttributeType ori(Orientation(), in[Position()].value(), Scale<>(), Radian());
        in.add(ori);

        ROS_DEBUG_STREAM_NAMED(transName, "VirtACC fits with in type: " << in);
        return {in};
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
