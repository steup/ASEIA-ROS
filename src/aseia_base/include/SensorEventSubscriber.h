#pragma once

#include <AbstractPubSub.h>
#include <Filter.h>

#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <aseia_base/SensorEvent.h>

#include <functional>


template<typename... Args>
class SensorEventSubscriber;

template<typename SensorEvent>
class SensorEventSubscriber<SensorEvent> : public AbstractPubSub<SensorEvent>  {
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
    mSub = ros::NodeHandle().subscribe(this->topic(), size, &SensorEventSubscriber::unpack, this, ros::TransportHints().udp());
    ros::TimerEvent time;
    this->publishType(time);
  }

  template<typename T>
  SensorEventSubscriber(void (T::*callback)(const SensorEvent&), T* objPtr, size_t size=1)
    : Base(Base::Type::subscriber), mCallback(std::bind(callback, objPtr, std::placeholders::_1))
  {
    mSub = ros::NodeHandle().subscribe(this->topic(), size, &SensorEventSubscriber::unpack, this);
    ros::TimerEvent time;
    this->publishType(time);
  }
};

template<typename SensorEvent, typename Filter>
class SensorEventSubscriber<SensorEvent, Filter> : public AbstractPubSub<SensorEvent>  {
 private:
    using Base = AbstractPubSub<SensorEvent>;

    ros::Subscriber       mSub;
    std::function<void (const SensorEvent&)> mCallback;
    void* mObjPtr;
    Filter mFilter;

    void unpack(const aseia_base::SensorEvent::ConstPtr& msg) {
      SensorEvent e;
      DeSerializer<decltype(msg->event.begin())> d(msg->event.begin(), msg->event.end());
      d >> e;
      if(mFilter(e))
        mCallback(e);
    }

public:
  SensorEventSubscriber(void (*callback)(const SensorEvent&), const Filter& filter, size_t size=1)
    : Base(Base::Type::subscriber), mCallback(callback),
      mFilter(filter)
  {
    mSub = ros::NodeHandle().subscribe(this->topic(), size, &SensorEventSubscriber::unpack, this);
    auto filterImpl = filter(::filter::s0);
    auto& filterBuf = this->mTypeMsg.filter;
    filterBuf.resize(filterImpl.size());
    Serializer<decltype(filterBuf.begin())> s(filterBuf.begin());
    s << filterImpl;
    ros::TimerEvent time;
    this->publishType(time);
  }

  template<typename T>
  SensorEventSubscriber(void (T::*callback)(const SensorEvent&), T* objPtr, const Filter& filter, size_t size=1)
    : Base(Base::Type::subscriber), mCallback(std::bind(callback, objPtr, std::placeholders::_1)),
      mFilter(filter)
  {
    mSub = ros::NodeHandle().subscribe(this->topic(), size, &SensorEventSubscriber::unpack, this);
    auto filterImpl = filter(::filter::s0);
    auto& filterBuf = this->mTypeMsg.filter;
    filterBuf.resize(filterImpl.size());
    Serializer<decltype(filterBuf.begin())> s(filterBuf.begin());
    s << filterImpl;
    ros::TimerEvent time;
    this->publishType(time);
  }
};
