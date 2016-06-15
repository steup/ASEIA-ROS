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

#include <aseia_base/SensorEvent.h>
#include <aseia_base/EventType.h>

template<typename SensorEvent>
class SensorEventSubscriber {
 private: 
    ros::Subscriber       mSub;
    void (*mCallback)(const SensorEvent&);
    
    
    constexpr std::string prefix() { return "/sensors"; }
    constexpr std::string managementTopic() { return prefix()+"/management"; }

    void unpack(const aseia_base::SensorEvent::ConstPtr& msg) {
      SensorEvent e;
      DeSerializer<decltype(msg->event.begin())> d(msg->event.begin(), msg->event.end());
      d >> e;
      mCallback(e);
    }

public:
  SensorEventSubscriber(void (*callback)(const SensorEvent&), unsigned int typePeriod = 1000)
  	: mCallback(callback)
    {
			SensorEvent e;
      aseia_base::EventType msg;
			std::ostringstream os;
      ros::NodeHandle n;
			ros::Publisher typePub = n.advertise<aseia_base::EventType>(managementTopic(), 1, true);

			msg.type = aseia_base::EventType::SUBSCRIBER;

			mFormat= FormatID(EventType(e))

			os << prefix() << "/" << EventID(e);
			msg.topic = os.str();

      EventType eType(e);

      Serializer<uint8_t*> sFormat((uint8_t*)&msg.format);
      sFormat << mFormat;

      msg.type.resize(eType.size());
      Serializer<decltype(msg.type.begin())> s(msg.data.begin());
      s << eType;

			os << "/" << mFormat;

      mSub = ros::NodeHandle().subscribe(os.str(), 1, &SensorEventSubscriber::unpack, this);
      ROS_INFO_STREAM("Sensor Publication Type: " << eType);
      mTypePub.publish(msg);
  }
  ~SensorEventSubscriber(){
  }
};
