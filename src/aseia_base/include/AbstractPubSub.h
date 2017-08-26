#pragma once

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <ros/console.h>

#include <aseia_base/EventType.h>

#include <EventType.h>
#include <IO.h>
#include <FormatID.h>
#include <EventID.h>

#include <functional>
#include <string>
#include <sstream>

template<typename Event>
class AbstractPubSub {
  protected:
    ros::Publisher        mTypePub;
    ros::Timer            mTimer;
    aseia_base::EventType mTypeMsg;
    enum class Type {
      publisher,
      subscriber
    };
    uint64_t              mId;
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

    uint64_t nodeId() const { return mId; }

  protected:
    AbstractPubSub(Type t, ros::Duration timeout = ros::Duration(60)) {
			std::ostringstream os;
      mId = std::hash<std::string>()(ros::this_node::getName());

			if(t == Type::publisher)
        mTypeMsg.type = aseia_base::EventType::PUBLISHER;
      else
        mTypeMsg.type = aseia_base::EventType::SUBSCRIBER;

      Event e;
      EventType eType = (EventType)e;

			mTypeMsg.topic = baseTopic();
      mTypeMsg.id = mId;

      mTypeMsg.data.resize(eType.size());
      Serializer<decltype(mTypeMsg.data.begin())> s(mTypeMsg.data.begin());
      s << eType;
      ros::NodeHandle n;
			mTypePub = n.advertise<aseia_base::EventType>(managementTopic(), 1, true);
      mTimer = n.createTimer(timeout, &AbstractPubSub::publishType, this);

      ROS_INFO_STREAM("Created " << (t==Type::publisher?"Publisher":"Subscriber") <<
                      " with EventType " << eType << " on Topic : " << topic());
    }

  public:

    void publishType(const ros::TimerEvent& e) const {
      ROS_DEBUG_STREAM("Publishing EventType");
      mTypePub.publish(mTypeMsg);
    }

};
