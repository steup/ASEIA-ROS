#pragma once

#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <ros/console.h>

#include <FormatID.h>
#include <EventID.h>
#include <EventType.h>
#include <IO.h>

#include <string>
#include <sstream>

#include <aseia_base/SensorEvent.h>
#include <aseia_base/EventType.h>

template<typename SensorEvent>
class SensorEventPublisher {
  private: 
    ros::Publisher        mPub; 
    
    constexpr std::string prefix() { return "/sensors"; }
    constexpr std::string managementTopic() { return prefix()+"/management"; }

public:
  SensorEventPublisher() {
      SensorEvent e;
      aseia_base::EventType msg;
			std::ostringstream os;
      ros::NodeHandle n;
			ros::Publisher typePub = n.advertise<aseia_base::EventType>(managementTopic(), 1, true);

			msg.type = aseia_base::EventType::PUBLISHER;

      EventType eType(e);

			os << prefix() << "/" << EventID(eType);
			msg.topic = os.str();


      msg.data.resize(eType.size());
      Serializer<decltype(msg.data.begin())> s(msg.data.begin());
      s << eType;

			os << "/" << FormatID(eType);

      mPub = ros::NodeHandle().advertise< aseia_base::SensorEvent >(os.str(), 1);
      ROS_INFO_STREAM("Sensor Publication Type: " << eType);
      typePub.publish(msg);

  }

  ~SensorEventPublisher(){

  }

  void publish(const SensorEvent& e) { 
    aseia_base::SensorEvent buffer;
    buffer.event.resize(SensorEvent::size());
    Serializer<std::vector<uint8_t>::iterator> s(buffer.event.begin());
    s << e;
    mPub.publish(buffer);
  }
};
