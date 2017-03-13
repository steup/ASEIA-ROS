#include <pluginlib/class_list_macros.h>
#include <Transformation.h>

#include <iostream>

namespace aseia_car_sim {

using namespace std;

class RoadTransformation : public Transformation {
  public:
    RoadTransformation() : Transformation(EventID::any) {}
    virtual ~RoadTransformation() {}
    virtual size_t arity() const { return 1; }
    virtual EventIDs in(EventID goal) const { return EventIDs({goal}); };
    virtual bool check(const EventType& out, const EventTypes& in) const { return false; }
    virtual TransPtr create(const EventType& out, const EventTypes& in) const { return TransPtr(); }
    virtual void print(ostream& o) const { o << "not yet implemented road to UTM transformation"; }
};

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::RoadTransformation, Transformation)
