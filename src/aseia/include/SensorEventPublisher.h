#pragma once

#include <ros/publisher.h>
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

#include <aseia/SensorEvent.h>
#include <aseia/EventType.h>

template<typename SensorEvent>
class SensorEventPublisher {
  private: 
    std::string        mTopic;
    FormatID           mFormat;
    aseia::EventType   mType;
    ros::Publisher     mPub; 
    std::atomic<bool>  mRunning;
    std::thread        mTypeThread;
    const unsigned int mTypePeriod;
    
    
    constexpr std::string prefix() { return "/sensors"; }
    constexpr std::string managementTopic() { return prefix()+"/management"; }
    void typeThreadFunc(){
      ros::NodeHandle n;
      ros::Publisher typePub = n.advertise<aseia::EventType>(managementTopic(), 10);
      while(ros::ok() && mRunning) {
        ROS_INFO_STREAM("publish: " << mType);
        typePub.publish(mType);
        std::this_thread::sleep_for(std::chrono::milliseconds(mTypePeriod));
      }
    }

public:
  SensorEventPublisher(const std::string& topic, uint16_t nodeID, unsigned int typePeriod = 1000)
    : mTopic(prefix() + "/" + topic),
      mFormat(nodeID, FormatID::Direction::publisher),
      mRunning(true),
      mTypeThread([this](){typeThreadFunc();}),
      mTypePeriod(typePeriod)
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
      mPub = ros::NodeHandle().advertise< aseia::SensorEvent >(mTopic+"/"+formatName.str(), 10);
      mRunning=true;
  }
  ~SensorEventPublisher(){
    mRunning=false;
    mTypeThread.join();
  }

  void publish(const SensorEvent& e) { 
    aseia::SensorEvent buffer;
    buffer.event.resize(SensorEvent::size());
    Serializer<std::vector<uint8_t>::iterator> s(buffer.event.begin(), buffer.event.end());
    s << e;
    mPub.publish(buffer);
  }
};
