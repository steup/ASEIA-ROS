#include <ros/ros.h>
#include <ros/console.h>

#include <EventType.h>
#include <FormatID.h>
#include <IO.h>

#include <aseia_base/EventType.h>
#include <aseia_base/SensorEvent.h>

#include <sstream>
#include <string>
#include <list>
#include <map>
#include <tuple>

using namespace std;
using Channel     = tuple< string, FormatID, FormatID >;
using EventTopic  = pair< string, FormatID >;
using FormatMap   = map< EventTopic, EventType >;
using NodeMap     = map< EventTopic, string >;
using FormatList  = list<FormatID>;
using Pubs        = map<string, FormatList>;
using Subs        = map<string, FormatList>;

const string&   topic   ( const Channel&    value) { return get<0>(value); }
const FormatID& pubId   ( const Channel&    value) { return get<1>(value); }
const FormatID& subId   ( const Channel&    value) { return get<2>(value); }
const string&   topic   ( const EventTopic& value) { return value.first;   }
const FormatID& formatId( const EventTopic& value) { return value.second;  }

FormatMap   formats;
NodeMap     nodes;
Pubs        pubs;
Subs        subs;

struct Forwarder{
  private:
    ros::Subscriber mSubscriber;
    ros::Publisher  mPublisher;
    const Channel&  mChannel;

    void forward(const aseia_base::SensorEvent::ConstPtr& msg){
      mPublisher.publish(*msg);
    }

    static string createPubTopic(const Channel& channel){
    	ostringstream out;
	out << pubId(channel);
	return topic(channel) + "/" + out.str();
    }
    
    static string createSubTopic(const Channel& channel){
    	ostringstream out;
	out << subId(channel);
	return topic(channel) + "/" + out.str();
    }
    
  public:
    Forwarder(const Channel& channel) 
      : mSubscriber(ros::NodeHandle().subscribe(createPubTopic(channel), 10, &Forwarder::forward, this)),
        mPublisher(ros::NodeHandle().advertise<aseia_base::SensorEvent>(createSubTopic(channel), 10)),
	mChannel(channel)
    {
    	ROS_INFO_STREAM("New channel: (" << topic(channel) << ", " << pubId(channel) << ", " << subId(channel) << ")");
    }
};

using ChannelMap = map<Channel, Forwarder>;

ChannelMap channels;

void handleEventType(const ros::MessageEvent<aseia_base::EventType const>& metaData) {

  const aseia_base::EventType::ConstPtr& msg = metaData.getMessage();
  EventType type;
  FormatID  format;

  ROS_INFO_STREAM("New node registered: " << metaData.getPublisherName());

  DeSerializer<decltype(msg->type.begin())> dType(msg->type.begin(), msg->type.end());
  dType >> type;
    
  DeSerializer<uint8_t*> dFormat((uint8_t*)&msg->format, (uint8_t*)&msg->format+sizeof(uint32_t));
  dFormat >> format;

  auto nodeIter   = nodes.insert  (make_pair(EventTopic(msg->topic, format), metaData.getPublisherName()));
  auto formatIter = formats.insert(make_pair(EventTopic(msg->topic, format), type));

  pair<map<string, FormatList>::iterator, bool> topicResult;

  if(format.direction() == FormatID::Direction::publisher)
    topicResult = pubs.insert(make_pair(msg->topic, FormatList()));
  else
    topicResult = subs.insert(make_pair(msg->topic, FormatList()));


  auto& topicFormatList = topicResult.first->second;
  auto  topicFormatIter  = find( topicFormatList.begin(), topicFormatList.end(), format);

  if(topicFormatIter == topicFormatList.end()) {
      topicFormatList.push_back(format);

      if(format.direction() == FormatID::Direction::publisher){
      	const auto& subFormats = subs.find(msg->topic);
	if(subFormats != subs.end())
      		for(const auto& subFormat : subFormats->second)
  			channels.emplace(piecewise_construct, 
					 make_tuple(msg->topic, format, subFormat),
					 make_tuple(Channel(msg->topic, format, subFormat))
					);
	}else{
      	const auto& pubFormats = pubs.find(msg->topic);
	if(pubFormats != pubs.end())
      		for(const auto& pubFormat : pubFormats->second)
      			channels.emplace(piecewise_construct, 
					 make_tuple(msg->topic, pubFormat, format),
					 make_tuple(Channel(msg->topic, pubFormat, format))
					);
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaEventCoupler");
  ros::NodeHandle n;
  ros::Subscriber typeSub  = n.subscribe("/sensors/management", 10, handleEventType);
  while(ros::ok())
    ros::spin();
  return 0;
};
