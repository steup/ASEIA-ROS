#pragma once

#include <AbstractPubSub.h>

#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <aseia_base/SensorEvent.h>

#include <functional>

template<typename SensorEvent>
class SensorEventSubscriber : public AbstractPubSub<SensorEvent>  {
 private:
    using Base = AbstractPubSub<SensorEvent>;

    ros::Subscriber       mSub;
    std::function<void (const SensorEvent&)> mCallback;
    void* mObjPtr;

    void unpack(const aseia_base::SensorEvent::ConstPtr& msg) {
      SensorEvent e;
      DeSerializer<decltype(msg->event.begin())> d(msg->event.begin(), msg->event.end());
      d >> e;
      mCallback(e);
    }

public:
  SensorEventSubscriber(void (*callback)(const SensorEvent&), size_t size=1)
    : Base(Base::Type::subscriber), mCallback(callback)
  {
    mSub = ros::NodeHandle().subscribe(this->topic(), size, &SensorEventSubscriber::unpack, this);
  }

  template<typename T>
  SensorEventSubscriber(void (T::*callback)(const SensorEvent&), T* objPtr, size_t size=1)
    : Base(Base::Type::subscriber), mCallback(std::bind(callback, objPtr, std::placeholders::_1))
  {
    mSub = ros::NodeHandle().subscribe(this->topic(), size, &SensorEventSubscriber::unpack, this);
  }
};
