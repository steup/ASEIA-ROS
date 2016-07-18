#pragma once

#include <ros/ros.h>

#include <iosfwd>
#include <memory>
#include <list>

class EventType;
class Transformation;

class Channel{
  private:
    const EventType& mOutType;
    const std::list<const EventType*>& mInTypes;
    ros::Publisher mPub;
    std::list<ros::Subscriber> mSubs;
    std::unique_ptr<Transformation> mTrans;
    void handleEvent(aseia::SensorEventConstPtr& msg);
  public:
    Channel(const std::list<const EventType*>& in, const EventType& out, TransID trans);
    const std::size_t inputSize() const { return mSubs.size(); }
    const EventType& outType() const { return mOutType; }
    const EventType& inType(std::size_t i) const;
    const Transformation& trans() const { return *mTrans; }
};

std::ostream operator<<(std::ostream& o, const Channel& c);
