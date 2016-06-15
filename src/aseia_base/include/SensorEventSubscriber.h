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
      EventType eType(e);

			os << prefix() << "/" << EventID(eType);
			msg.topic = os.str();


      msg.data.resize(eType.size());
      Serializer<decltype(msg.data.begin())> s(msg.data.begin());
      s << eType;

			os << "/" << FormatID(eType);

      mSub = ros::NodeHandle().subscribe(os.str(), 1, &SensorEventSubscriber::unpack, this);
      ROS_INFO_STREAM("Sensor Subscription Type: " << eType);
      typePub.publish(msg);
  }
  ~SensorEventSubscriber(){
  }
};
