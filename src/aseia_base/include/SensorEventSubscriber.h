#pragma once

#include <AbstractPubSub.h>

#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <aseia_base/SensorEvent.h>

template<typename SensorEvent>
class SensorEventSubscriber : public AbstractPubSub<SensorEvent>  {
 private:
    using Base = AbstractPubSub<SensorEvent>;

    ros::Subscriber       mSub;
    void (*mCallback)(const SensorEvent&);

    void unpack(const aseia_base::SensorEvent::ConstPtr& msg) {
      SensorEvent e;
      DeSerializer<decltype(msg->event.begin())> d(msg->event.begin(), msg->event.end());
      d >> e;
      mCallback(e);
    }

public:
  SensorEventSubscriber(void (*callback)(const SensorEvent&))
    : Base(Base::Type::subscriber), mCallback(callback)
  {
    mSub = ros::NodeHandle().subscribe(this->topic(), 1, &SensorEventSubscriber::unpack, this);
  }
};
