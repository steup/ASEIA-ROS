#pragma once

#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <ros/console.h>

#include <aseia_base/EventType.h>

#include <EventType.h>
#include <IO.h>
#include <FormatID.h>
#include <EventID.h>

#include <string>
#include <sstream>

template<typename Event>
class AbstractPubSub {
  private:
    ros::Publisher        mTypePub;
    ros::Timer            mTimer;
    aseia_base::EventType mTypeMsg;
  protected:
    enum class Type {
      publisher,
      subscriber
    };
  public:
    constexpr std::string prefix() {
      return "/sensors";
    }

    constexpr std::string managementTopic() {
      return prefix()+"/management";
    }

    std::string baseTopic() const {
      EventType eT = (EventType)Event();
      std::ostringstream os;
      os << prefix() << "/" << EventID(eT);
      return os.str();
    }

    std::string topic() const {
      EventType eT = (EventType)Event();
      std::ostringstream os;
      os << baseTopic() << "/" << FormatID(eT);
      return os.str();
    }

  protected:
    AbstractPubSub(Type t, ros::Duration timeout = ros::Duration(60)) {
			std::ostringstream os;

			if(t == Type::publisher)
        mTypeMsg.type = aseia_base::EventType::PUBLISHER;
      else
        mTypeMsg.type = aseia_base::EventType::SUBSCRIBER;

      Event e;
      EventType eType = (EventType)e;

			mTypeMsg.topic = baseTopic();

      mTypeMsg.data.resize(eType.size());
      Serializer<decltype(mTypeMsg.data.begin())> s(mTypeMsg.data.begin());
      s << eType;
      ros::NodeHandle n;
			mTypePub = n.advertise<aseia_base::EventType>(managementTopic(), 1, true);

      ROS_INFO_STREAM("Created " << (t==Type::publisher?"Publisher":"Subscriber") <<
                      " with EventType " << eType << " on Topic : " << topic());
      publishType();
    }

  public:

    void publishType() const {
      ROS_DEBUG_STREAM("Publishing EventType");
      mTypePub.publish(mTypeMsg);
    }

};
