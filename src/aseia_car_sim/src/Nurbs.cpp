#include "Attributes.h"

#include <ros/ros.h>
#include <BaseEvent.h>
#include <ID.h>
#include <Unit.h>
#include <SensorEventPublisher.h>

using namespace ::id::attribute;
using namespace std;

namespace aseia_car_sim {

class NurbsPublisher {
  private:
    struct NurbsBaseConfig : public BaseConfig {
      using TimeValueType = Value<double, 1>;
      using PositionValueType = Value<float, 3>;
      using PositionScale = Scale<std::ratio<1>, 1>;
    };
    using Ref = Attribute<Reference, Value<float, 3>, Meter>;
    using NurbData = Attribute<Nurbs, Value<float, 1, 1, false>, Dimensionless>;
    using NurbsReference = BaseEvent<NurbsBaseConfig>
                            ::append<Ref>::type
                            ::append<NurbData>::type;
    ros::Timer mTimer;
    SensorEventPublisher<NurbsReference> mPub;
    NurbsReference mRef;

    void periodic(const ros::TimerEvent& e){
      mRef.attribute(Time()).value()={{{ros::Time::now().toSec(), 0.0}}};
      mPub.publish(mRef);
    }
  public:
    NurbsPublisher()
      : mTimer(ros::NodeHandle().createTimer(ros::Duration(60), &NurbsPublisher::periodic, this))
    {
      //mRef.attribute(Position()).value()={{{}}};
      //mRef.attribute(Reference()).value()={{{}}};
      //mRef.attribute(Nurbs()).value()={{{}}};
    }
};
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "nurbs");
  aseia_car_sim::NurbsPublisher nurbs;
  while(ros::ok())
    ros::spin();
  return 0;
}
