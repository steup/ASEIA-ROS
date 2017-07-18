#pragma once

#include <AbstractPubSub.h>

#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <aseia_base/SensorEvent.h>
#include <ID.h>

template<typename SensorEvent>
class SensorEventPublisher : public AbstractPubSub<SensorEvent>{
  private:
    using Base = AbstractPubSub<SensorEvent>;
    ros::Publisher        mPub;

  public:
    SensorEventPublisher(size_t size=1, bool latch=false)
      : Base(Base::Type::publisher) {
        mPub = ros::NodeHandle().advertise< aseia_base::SensorEvent >(this->topic(), size, latch);
    }

    void publish(SensorEvent& e) {
      aseia_base::SensorEvent buffer;
      buffer.event.resize(SensorEvent::size());
      Serializer<std::vector<uint8_t>::iterator> s(buffer.event.begin());
      s << e;
      mPub.publish(buffer);
    }
};
