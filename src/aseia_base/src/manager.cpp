#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>

#include <aseia_base/SensorEvent.h>
#include <aseia_base/EventType.h>
#include <aseia_base/Publishers.h>

#include <Transformation.h>
#include <Transformations.h>
#include <KnowledgeBase.h>
#include <Channel.h>
#include <MetaEvent.h>
#include <MetaFactory.h>
#include <FormatID.h>
#include <EventID.h>
#include <IO.h>
#include <Prime.h>

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
using namespace id::attribute;


static string topic(EventID eID, FormatID fID) {
  ostringstream os;
  os << "/sensors/" << eID << "/" << fID;
  return os.str();
}

static string topic(const EventType& eT) { return topic(eT, eT); }

class RosChannel : public Channel {
  protected:
    ros::Publisher mPub;
    list<ros::Subscriber> mSubs;
    int mNodeID=0;
    MetaValue mPrimeID;

  public:
    void unpackEvent(aseia_base::SensorEvent::ConstPtr msgPtr, const EventType& eT) {
        MetaEvent  e(eT);
        DeSerializer<decltype(msgPtr->event.begin())> d(msgPtr->event.begin(), msgPtr->event.end());
        d >> e;
        if(((int)e[PublisherID()].value().get(0,0).value())%PrimeGenerator::prime(mNodeID)!=0)
          handleEvent(e);
        else
          ROS_DEBUG_STREAM("Rejecting cyclic event: " << e);
    }

    RosChannel() = default;

    RosChannel(TransPtr&& trans) : Channel(move(trans)) {
      ros::NodeHandle n;
      ros::NodeHandle("~").getParam("id", mNodeID);
      mPrimeID = MetaValue(PrimeGenerator::prime(mNodeID), mTrans->out()[PublisherID()].value().typeId());
      if(mNodeID)
        ROS_DEBUG_STREAM("Establishing Channel with node id " << mNodeID << "and prime  " << mPrimeID);
      else
        ROS_ERROR_STREAM("Channel creation failed node id is 0");
      mPub =  n.advertise<aseia_base::SensorEvent>(topic(mTrans->out()), 1, true);
      for(const EventType& in : mTrans->in()) {
        for(const EventType& subType : KnowledgeBase::findCompatible(in)) {
          ROS_DEBUG_STREAM("Subscribing to topic: " << topic(subType));
          mSubs.push_back(n.subscribe<aseia_base::SensorEvent>(topic(subType), 1, boost::bind(&RosChannel::unpackEvent,  this,  _1, subType), ros::VoidConstPtr(), ros::TransportHints().udp()));
        }
      }
    }

    virtual void error(Errors error, const MetaEvent& e) const {
      std::string errorStr;
      switch(error){
        case(Errors::InvalidEvent)    : errorStr = "Event invalid";
                                       break;
        case(Errors::MissingAttribute): errorStr = "Event incomplete";
                                       break;
        case(Errors::IncompatibleType): errorStr = "Event type inconvertible to target Event Type";
                                       break;
        default                       : errorStr = "Unknown error";
      }
      ROS_ERROR_STREAM_NAMED("ros_channel", *this << " expecting event of type:\n" << mTrans->out() << "Error: " << errorStr << " on generated Event\n" << e << "\nof type\n" << (EventType)e);
    }

    virtual void publishEvent(MetaEvent& e) const {
      e[PublisherID()].value()*=mPrimeID;
      aseia_base::SensorEvent sE;
      Serializer<decltype(back_inserter(sE.event))> s(back_inserter(sE.event));
      s << e;
      mPub.publish(sE);
    }
};

class ChannelManager {
  private:
    using ChannelRegistry = std::multimap<pair<EventID, FormatID>, RosChannel>;
    using ConstIter = ChannelRegistry::const_iterator;
    using Iter = ChannelRegistry::const_iterator;
    using Value = ChannelRegistry::value_type;
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
    ros::ServiceServer mTransSrv;
    ChannelRegistry mChannels;
    TransformationLoaderPtr mTransLoaderPtr;
    vector<TransformationPtr> mTransList;
  public:
    bool provideChannels(aseia_base::Publishers::Request& req, aseia_base::Publishers::Response& res) {
      ostringstream os;
      for(const Value& v: mChannels)
        os << v.second << endl;
      res.publishers = os.str();
      return true;
    }

    bool transforms(aseia_base::Publishers::Request& req, aseia_base::Publishers::Response& res) {
      ostringstream os;
      KnowledgeBase::print(os);
      res.publishers = os.str();
      return true;
    }

    void handleNode(aseia_base::EventTypeConstPtr eTPtr) {
      EventType eT;
      DeSerializer<decltype(eTPtr->data.begin())> d(eTPtr->data.begin(), eTPtr->data.end());
      d >> eT;


      if(eTPtr->type == aseia_base::EventType::PUBLISHER) {
          ROS_DEBUG_STREAM("Publisher: " << eT);
          KnowledgeBase::registerEventType(eT);
          // Scan all subscribers for new transformations
      } else {
        MetaFilter filter({&eT});
        if(eTPtr->filter.empty())
          filter = MetaFilter();
        else {
          d = DeSerializer<decltype(eTPtr->filter.begin())>(eTPtr->filter.begin(), eTPtr->filter.end());
          try {
            d >> filter;
          }catch(const MetaFilterError& e) {
            ROS_ERROR_STREAM("Error " << e.what() << " deserializing MetaFilter with EventType: \n" <<
                             eT << "\n Received Buffer: " << eTPtr->filter);
            filter = MetaFilter();
          }
        }
        ROS_DEBUG_STREAM("Subscriber: " << eT);
        ROS_DEBUG_STREAM("Searching for Transforms");
        for(const CompositeTransformation& cT : KnowledgeBase::findTransforms(eT, filter)) {

          ROS_DEBUG_STREAM("Potential Transform: " << cT);

          auto comp = [&cT](const Value& v){return v.second.trans() && *v.second.trans() == cT;};

          auto range = mChannels.equal_range(make_pair(EventID(eT), FormatID(eT)));
          if(none_of(range.first, range.second, comp)) {
            auto it = mChannels.emplace(make_pair(EventID(eT), FormatID(eT)), cT.create(AbstractPolicy()));
            ROS_INFO_STREAM("Established Channel" << it->second);
          } else
            ROS_DEBUG_STREAM("Channel already existing");
        }
      }
    }

    ChannelManager() {
      ROS_INFO_STREAM("Starting Channel Manager!");
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
            ROS_ERROR_STREAM("Loaded and registered " << transName);
          } catch(const pluginlib::LibraryLoadException& e) {
            ROS_ERROR_STREAM("Error loading library for transformation " << transName << ": " << e.what());
          } catch(const pluginlib::CreateClassException& e) {
            ROS_ERROR_STREAM("Error creating transformation " << transName << ": " << e.what());
          }
      }
      ros::NodeHandle n;
      mSub = n.subscribe("/sensors/management", 100, &ChannelManager::handleNode, this);
      mPubSrv = n.advertiseService("/sensors/channels", &ChannelManager::provideChannels, this);
      mTransSrv = n.advertiseService("/sensors/transforms", &ChannelManager::transforms, this);
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaChannelManager");
  ChannelManager manager;
  while(ros::ok())
    ros::spin();
  return 0;
};
