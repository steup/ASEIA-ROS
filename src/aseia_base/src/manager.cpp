#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>

#include <aseia_base/SensorEvent.h>
#include <aseia_base/EventType.h>
#include <aseia_base/Publishers.h>

#include <Transformation.h>
#include <Transformations.h>
#include <KnowledgeBase.h>
#include <AbstractRegistry.h>
#include <Channel.h>
#include <MetaEvent.h>
#include <MetaFactory.h>
#include <FormatID.h>
#include <EventID.h>
#include <IO.h>

#include <boost/algorithm/string/split.hpp>

#include <sstream>
#include <string>
#include <list>
#include <iterator>
#include <memory>

using namespace std;
using boost::split;
using boost::token_compress_on;
using boost::is_any_of;

string topic(EventID eID, FormatID fID) {
  ostringstream os;
  os << "/sensors/" << eID << "/" << fID;
  return os.str();
}

string topic(const EventType& eT) { return topic(eT, eT); }

/*bool topic(const string& s, EventID& eID, FormatID& fID) {
  istringstream is(s);
  is >> "/sensors/" >> eID >> "/" >> fID;
  return true;
}*/

class RosChannel : public Channel {
  protected:
    ros::Publisher mPub;
    list<ros::Subscriber> mSubs;

  public:
    void unpackEvent(aseia_base::SensorEvent::ConstPtr msgPtr, const EventType& eT) {
        MetaEvent  e(eT);
        DeSerializer<decltype(msgPtr->event.begin())> d(msgPtr->event.begin(), msgPtr->event.end());
        d >> e;
        ROS_DEBUG_STREAM("Got Event to transform: " << e);
        handleEvent(e);
    }

    RosChannel() = default;

    RosChannel(TransPtr&& trans) : Channel(move(trans)) {
      ros::NodeHandle n;
      mPub =  n.advertise<aseia_base::SensorEvent>(topic(mTrans->out()), 1);
      for(const EventType& in : mTrans->in()) {
        mSubs.push_back(n.subscribe<aseia_base::SensorEvent>(topic(in), 1, boost::bind(&RosChannel::unpackEvent,  this,  _1, in)));
      }
    }

    RosChannel(RosChannel&& movee) : Channel(move(movee)), mPub(movee.mPub){
      ros::NodeHandle n;
      for(const EventType& in : mTrans->in()) {
        mSubs.push_back(n.subscribe<aseia_base::SensorEvent>(topic(in), 1, boost::bind(&RosChannel::unpackEvent,  this,  _1, in)));
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
    using TransformationLoader = pluginlib::ClassLoader<Transformation>;
    using TransformationLoaderPtr = unique_ptr<TransformationLoader>;
    struct Deleter {
      TransformationLoader& loader;
      string lookupName;
      Deleter(TransformationLoader& loader, string lookupName)
        : loader(loader), lookupName(lookupName) {}
      void operator()(Transformation* transPtr) {
        delete transPtr;
        loader.unloadLibraryForClass(lookupName);
      }
    };
    using TransformationPtr = unique_ptr<Transformation, Deleter>;
    ros::Subscriber mSub;
    ros::ServiceServer mPubSrv;
    ChannelRegistry mChannels;
    TransformationLoaderPtr mTransLoaderPtr;
    vector<TransformationPtr> mTransList;
  public:
    bool providePublishers(aseia_base::Publishers::Request& req, aseia_base::Publishers::Response& res) {
      ostringstream os;
      os << "Not yet implemented";
      //for(const EventType& eT : mPubs)
      //  os << topic(eT) << ":" << endl << eT;
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
        for(const CompositeTransformation& t : KnowledgeBase::findTransforms(eT)) {
            ROS_DEBUG_STREAM("Potential Transform: " << t);
            auto comp = [&t](const RosChannel& c){return c.trans() && t==*c.trans();};
            if(none_of(mChannels.find(eT).begin(), mChannels.find(eT).end(), comp)) {
              RosChannel c(t.create(AbstractPolicy()));
              ROS_INFO_STREAM("Established Channel" << c);
              mChannels.registerType(eT, move(c));
            } else
              ROS_DEBUG_STREAM("Channel already existing");
        }
      }
    }
    ChannelManager() {
      KnowledgeBase::registerTransformation(cast);
      KnowledgeBase::registerTransformation(rescale);
      string transforms;
      if( ros::param::get("~transformations", transforms)) {
        mTransLoaderPtr.reset(new TransformationLoader("aseia_base", "Transformation"));
        vector<string> transNameList;
        boost::split(transNameList, transforms, is_any_of(", "), token_compress_on);
        for(const string& transName : transNameList)
          try {
            mTransList.emplace_back(mTransLoaderPtr->createUnmanagedInstance(transName), Deleter(*mTransLoaderPtr, transName));
            KnowledgeBase::registerTransformation(*mTransList.back());
          } catch(const pluginlib::LibraryLoadException& e) {
            ROS_ERROR_STREAM("Error loading library for transformation " << transName << ": " << e.what());
          } catch(const pluginlib::CreateClassException& e) {
            ROS_ERROR_STREAM("Error creating transformation " << transName << ": " << e.what());
          }
      }
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
