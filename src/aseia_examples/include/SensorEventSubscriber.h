#pragma once

#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <ros/console.h>

#include <FormatID.h>
#include <EventType.h>
#include <IO.h>

#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>

#include <aseia_msgs//SensorEvent.h>
#include <aseia_msgs/EventType.h>

template<typename SensorEvent>
class SensorEventSubscriber {
 private: 
    std::string           mTopic;
    FormatID              mFormat;
    aseia_msgs::EventType mType;
    ros::Subscriber       mSub;
    std::thread           mTypeThread;
    std::atomic<bool>     mRunning;
    const unsigned int mTypePeriod;
    void (*mCallback)(const SensorEvent&);
    
    
    constexpr std::string prefix() { return "/sensors"; }
    constexpr std::string managementTopic() { return prefix()+"/management"; }

    void typeThreadFunc(){
      ros::NodeHandle n;
      ros::Publisher typePub = n.advertise<aseia_msgs::EventType>(managementTopic(), 10);
      while(ros::ok() && mRunning.load()) {
        ROS_INFO_STREAM("publish: " << mType);
        typePub.publish(mType);
        std::this_thread::sleep_for(std::chrono::milliseconds(mTypePeriod));
      }
    }

    void unpack(const aseia_msgs::SensorEvent::ConstPtr& msg) {
      SensorEvent e;
      DeSerializer<decltype(msg->event.begin())> d(msg->event.begin(), msg->event.end());
      d >> e;
      mCallback(e);
    }

public:
  SensorEventSubscriber(const std::string& topic, uint16_t nodeID, void (*callback)(const SensorEvent&), unsigned int typePeriod = 1000)
    : mTopic(prefix() + "/" + topic),
      mFormat(nodeID, FormatID::Direction::subscriber),
      mTypeThread([this](){typeThreadFunc();}),
      mTypePeriod(typePeriod),
      mCallback(callback)
    {
      SensorEvent e;
      EventType eType(e);

      Serializer<uint8_t*> sFormat((uint8_t*)&mType.format, (uint8_t*)&mType.format+sizeof(uint32_t));
      sFormat << mFormat;

      mType.topic = mTopic;

      mType.type.resize(eType.size());
      Serializer<decltype(mType.type.begin())> s(mType.type.begin(), mType.type.end());
      s << eType;

      std::ostringstream formatName;
      formatName << mFormat;

      ros::NodeHandle n;
      mSub = ros::NodeHandle().subscribe< aseia_msgs::SensorEvent >(mTopic+"/"+formatName.str(), 10, &SensorEventSubscriber::unpack, this);
      mRunning=true;
  }
  ~SensorEventSubscriber(){
    mRunning=false;
    mTypeThread.join();
  }
};
