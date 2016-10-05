#include <ros/ros.h>
#include <ros/console.h>

#include <aseia_base/SensorEvent.h>
#include <aseia_base/EventType.h>
#include <aseia_base/Publishers.h>

#include <Transformation.h>
#include <KnowledgeBase.h>
#include <AbstractRegistry.h>
#include <Channel.h>
#include <MetaEvent.h>
#include <MetaFactory.h>
#include <FormatID.h>
#include <EventID.h>
#include <IO.h>

#include <sstream>
#include <string>
#include <list>
#include <iterator>

std::string topic(EventID eID, FormatID fID) {
  std::ostringstream os;
  os << "/sensors/" << eID << "/" << fID;
  return os.str();
}

std::string topic(const EventType& eT) { return topic(eT, eT); }

/*bool topic(const std::string& s, EventID& eID, FormatID& fID) {
  std::istringstream is(s);
  is >> "/sensors/" >> eID >> "/" >> fID;
  return true;
}*/

class RosChannel : public Channel {
  protected:
    ros::Publisher mPub;
    std::list<ros::Subscriber> mSubs;

  public:
    void unpackEvent(aseia_base::SensorEvent::ConstPtr msgPtr, const EventType* eTPtr) {
        MetaEvent  e(*eTPtr);
        DeSerializer<decltype(msgPtr->event.begin())> d(msgPtr->event.begin(), msgPtr->event.end());
        d >> e;
        ROS_DEBUG_STREAM("Got Event to transform: " << e);
        handleEvent(e);
    }

    RosChannel() = default;

    RosChannel(TransPtr&& trans) : Channel(std::move(trans)) {
      ros::NodeHandle n;
      mPub =  n.advertise<aseia_base::SensorEvent>(topic(mTrans->out()), 1);
      for(const EventType* in : mTrans->in()) {
        mSubs.push_back(n.subscribe<aseia_base::SensorEvent>(topic(*in), 1, boost::bind(&RosChannel::unpackEvent,  this,  _1, in)));
      }
    }

    RosChannel(RosChannel&& movee) : Channel(std::move(movee)), mPub(movee.mPub){
      ros::NodeHandle n;
      for(const EventType* in : mTrans->in()) {
        mSubs.push_back(n.subscribe<aseia_base::SensorEvent>(topic(*in), 1, boost::bind(&RosChannel::unpackEvent,  this,  _1, in)));
      }
    }

    virtual void publishEvent(const MetaEvent& e) const {
      ROS_DEBUG_STREAM("Sending transformed Event: " << e);
      aseia_base::SensorEvent sE;
      Serializer<decltype(back_inserter(sE.event))> s(back_inserter(sE.event));
      s << e;
      mPub.publish(sE);
    }
};

class ChannelManager {
  private:
    using ChannelRegistry = AbstractRegistry<RosChannel>;
    ros::Subscriber mSub;
    ros::ServiceServer mPubSrv;
    ChannelRegistry mChannels;
  public:
    bool providePublishers(aseia_base::Publishers::Request& req, aseia_base::Publishers::Response& res) {
      std::ostringstream os;
      os << "Not yet implemented";
      //for(const EventType& eT : mPubs)
      //  os << topic(eT) << ":" << std::endl << eT;
      res.publishers = os.str();
      return true;
    }

    void handleNode(aseia_base::EventTypeConstPtr eTPtr) {
      EventType eT;
      DeSerializer<decltype(eTPtr->data.begin())> d(eTPtr->data.begin(), eTPtr->data.end());
      d >> eT;
      //ROS_INFO_STREAM("Got new " << (eTPtr->type==aseia_base::EventType::PUBLISHER?"Publisher":"Subscriber") <<
      //                " with EventType " << eT << "on base topic " << eTPtr->topic);
      if(eTPtr->type == aseia_base::EventType::PUBLISHER) {
          ROS_DEBUG_STREAM("Publisher: " << eT);
          KnowledgeBase::registerEventType(eT);
      } else {
        //TODO: pass Transformation and EventLists do discard existing transformations
        for(const ConfiguredTransformation& t : KnowledgeBase::findTransforms(eT)) {
            ROS_DEBUG_STREAM("Potential Transform: " << t);
            auto comp = [&t](const RosChannel& c){return c.trans() && t==*c.trans();};
            if(none_of(mChannels.find(eT).begin(), mChannels.find(eT).end(), comp)) {
              RosChannel c(t.create());
              ROS_INFO_STREAM("Established Channel" << c);
              mChannels.registerType(eT, std::move(c));
            } else
              ROS_DEBUG_STREAM("Channel already existing");
        }
      }
    }
    ChannelManager() {
      ros::NodeHandle n;
      mSub = n.subscribe("/sensors/management", 100, &ChannelManager::handleNode, this);
      mPubSrv = n.advertiseService("/sensors/publishers", &ChannelManager::providePublishers, this);
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaChannelManager");
  ChannelManager manager;
  while(ros::ok())
    ros::spin();
  return 0;
};
