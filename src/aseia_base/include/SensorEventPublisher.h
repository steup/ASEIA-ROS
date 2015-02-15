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

#include <aseia_base/SensorEvent.h>
#include <aseia_base/EventType.h>

template<typename SensorEvent>
class SensorEventPublisher {
  private: 
    std::string           mTopic;
    FormatID              mFormat;
    ros::Publisher        mPub; 
    ros::Publisher        mTypePub; 
    
    constexpr std::string prefix() { return "/sensors"; }
    constexpr std::string managementTopic() { return prefix()+"/management"; }

public:
  SensorEventPublisher(const std::string& topic, uint16_t nodeID)
    : mTopic(prefix() + "/" + topic),
      mFormat(nodeID, FormatID::Direction::publisher)
    {
      SensorEvent e;
      EventType eType(e);

      aseia_base::EventType msg;
      msg.topic = mTopic;

      Serializer<uint8_t*> sFormat((uint8_t*)&msg.format, (uint8_t*)&msg.format+sizeof(uint32_t));
      sFormat << mFormat;

      msg.type.resize(eType.size());
      Serializer<decltype(msg.type.begin())> s(msg.type.begin(), msg.type.end());
      s << eType;

      std::ostringstream formatName;
      formatName << mFormat;

      ros::NodeHandle n;
      mPub = ros::NodeHandle().advertise< aseia_base::SensorEvent >(mTopic+"/"+formatName.str(), 10);
      mTypePub = n.advertise<aseia_base::EventType>(managementTopic(), 10, true);
      ROS_INFO_STREAM("Sensor Publication Type: " << eType);
      mTypePub.publish(msg);

  }

  SensorEventPublisher(){
    
  }

  ~SensorEventPublisher(){

  }

  void publish(const SensorEvent& e) { 
    aseia_base::SensorEvent buffer;
    buffer.event.resize(SensorEvent::size());
    Serializer<std::vector<uint8_t>::iterator> s(buffer.event.begin(), buffer.event.end());
    s << e;
    mPub.publish(buffer);
  }
};
