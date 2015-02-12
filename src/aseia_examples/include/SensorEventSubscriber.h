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

#include <aseia_msgs//SensorEvent.h>
#include <aseia_msgs/EventType.h>

template<typename SensorEvent>
class SensorEventSubscriber {
 private: 
    std::string           mTopic;
    FormatID              mFormat;
    ros::Subscriber       mSub;
    ros::Publisher        mTypePub;
    void (*mCallback)(const SensorEvent&);
    
    
    constexpr std::string prefix() { return "/sensors"; }
    constexpr std::string managementTopic() { return prefix()+"/management"; }

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
      mCallback(callback)
    {
      SensorEvent e;
      EventType eType(e);

      aseia_msgs::EventType msg;
      msg.topic = mTopic;

      Serializer<uint8_t*> sFormat((uint8_t*)&msg.format, (uint8_t*)&msg.format+sizeof(uint32_t));
      sFormat << mFormat;

      msg.type.resize(eType.size());
      Serializer<decltype(msg.type.begin())> s(msg.type.begin(), msg.type.end());
      s << eType;

      std::ostringstream formatName;
      formatName << mFormat;

      ros::NodeHandle n;
      mSub = ros::NodeHandle().subscribe< aseia_msgs::SensorEvent >(mTopic+"/"+formatName.str(), 10, &SensorEventSubscriber::unpack, this);
      mTypePub = n.advertise<aseia_msgs::EventType>(managementTopic(), 10, true);
      ROS_INFO_STREAM("publish: " << eType);
      mTypePub.publish(msg);
  }
  ~SensorEventSubscriber(){
  }
};
