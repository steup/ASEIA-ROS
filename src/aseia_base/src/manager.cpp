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

using TypeName = string;
using NodeName = string;
struct FormattedType {
	const string mTypeName;
	const FormatID mFormatID;
	const EventType mType;
	FormattedType(const string& typeName, const FormatID& formatID, const EventType& type)
		: mTypeName(typeName), mFormatID(formatID), mType(type)
	{};
	bool isCompatible(const FormattedType& b) const{
		return mFormatID.direction() != b.mFormatID.direction() && mType == b.mType && mTypeName == mTypeName;
	}
	bool hasDifferentFormats(const FormattedType& b) const{
		return mFormatID.direction() != b.mFormatID.direction() && mTypeName == b.mTypeName && !(mType == b.mType);
	}
	bool hasDifferentTypes(const FormattedType& b) const{
		return mFormatID.direction() != b.mFormatID.direction() && mTypeName != b.mTypeName && !(mType == b.mType);
	}
	bool isPublisher() const{
		return mFormatID.direction() == FormatID::Direction::publisher;
	}
};
using FormatList = list<FormattedType>;

struct Channel {
	const FormattedType& mPubType;
	const FormattedType& mSubType;
	Channel(const FormattedType& pubType, const FormattedType& subType)
		: mPubType(pubType), mSubType(subType)
	{}
	bool operator<(const Channel& b) const{
		if(mPubType.mTypeName <  b.mPubType.mTypeName)
			return true;
		if(mPubType.mTypeName >  b.mPubType.mTypeName)
			return false;
		if(mSubType.mTypeName <  b.mSubType.mTypeName)
			return true;
		return false;
	}
};

// \brief central data structure to hold P/S information //
using NodeMap   = map<NodeName, FormatList>;

struct EventHandler{
	protected:
    ros::Subscriber mSubscriber;
    ros::Publisher  mPublisher;
    const NodeName& mPubName;
    const NodeName& mSubName;

    static string createPubTopic(const Channel& channel){
    	ostringstream out;
			out << channel.mPubType.mFormatID;
			return channel.mPubType.mTypeName + "/" + out.str();
    }
    
    static string createSubTopic(const Channel& channel){
    	ostringstream out;
			out << channel.mSubType.mFormatID;
			return channel.mSubType.mTypeName + "/" + out.str();
    }

		virtual void handle(const aseia_base::SensorEvent::ConstPtr& msg) = 0;
	private:
		void incomingEvent(const aseia_base::SensorEvent::ConstPtr& msg){
			handle(msg);
		}

	public:
    EventHandler(const Channel& channel, const NodeName& pubName, const NodeName& subName) 
      : mSubscriber(ros::NodeHandle().subscribe(createPubTopic(channel), 10, &EventHandler::incomingEvent, this)),
        mPublisher(ros::NodeHandle().advertise<aseia_base::SensorEvent>(createSubTopic(channel), 10)),
				mPubName(pubName),
				mSubName(subName)
    {
    	ROS_INFO_STREAM("New channel: (" << channel.mPubType.mTypeName << ": " << mPubName << "-> " << mSubName << ")");
    }
		
};

struct Forwarder: public EventHandler{
	private:
		virtual void handle(const aseia_base::SensorEvent::ConstPtr& msg){
			mPublisher.publish(*msg);
		}
  public:
    Forwarder(const Channel& channel, const NodeName& pubName, const NodeName& subName) 
      : EventHandler(channel, pubName, subName)
    {}
};

struct AttributeTransformer : public EventHandler{
	private:
//		Transformation trans;

		virtual void handle(const aseia_base::SensorEvent::ConstPtr& msg){
			//MetaEvent e;
  		//DeSerializer<decltype(msg->type.begin())> dE(msg->event.begin(), msg->event.end());
  		//dE >> e;
			//trans(e);
			aseia_base::SensorEvent newMsg;
      //newMsg.event.resize(e.size());
      //Serializer<decltype(newMsg.event.begin())> s(newMsg.event.begin(), newMsg.event.end());
      //s << e;
			//mPublisher.publish(newMsg);
			ROS_ERROR_STREAM("transformation not implemented yet");
		}
  public:
    AttributeTransformer(const Channel& channel, const NodeName& pubName, const NodeName& subName) 
      : EventHandler(channel, pubName, subName)/*,
				trans(channel.mPubFormat.mType, channel.mSubFormat.mType)*/
    {}
};

using SimpleChannelMap = map<Channel, Forwarder>;
using FormatTransformChannelMap = map<Channel, AttributeTransformer>;
SimpleChannelMap simpleChannels;
FormatTransformChannelMap attrTransChannels;
NodeMap nodes;

void unpackEventTypeInfo(const aseia_base::EventType::ConstPtr& msg, EventType& type, FormatID& format){
  DeSerializer<decltype(msg->type.begin())> dType(msg->type.begin(), msg->type.end());
  dType >> type;
	
  DeSerializer<uint8_t*> dFormat((uint8_t*)&msg->format, (uint8_t*)&msg->format+sizeof(uint32_t));
  dFormat >> format;

}

const string& nodeName(NodeMap::iterator iter) { return iter->first; }
FormatList& nodeFormatList(NodeMap::iterator iter) { return iter->second; }

void handleEventType(const ros::MessageEvent<aseia_base::EventType const>& metaData) {

  EventType type;
  FormatID  format;

  unpackEventTypeInfo(metaData.getMessage(), type, format);

	const NodeName& newNodeName = metaData.getPublisherName();
	const FormattedType newFormattedType = FormattedType(metaData.getMessage()->topic, format, type);
  auto nodeResult = nodes.insert(make_pair(newNodeName, FormatList()));
	auto nodeIter   = nodeResult.first;
	nodeFormatList(nodeIter).emplace_back(newFormattedType);
  
	ROS_INFO_STREAM("New node registered: " << metaData.getPublisherName());

	for(const auto& node : nodes)	{
		const auto& nodeName = node.first;
		const auto& formatList = node.second;
		for(const auto& format : formatList) {
			if(newFormattedType.isCompatible(format)) {
				if(newFormattedType.isPublisher()) {
					const Channel newChannel = Channel(newFormattedType, format);
					simpleChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, newNodeName, nodeName));
				} else {
					const Channel newChannel = Channel(format, newFormattedType);
					simpleChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, nodeName, newNodeName));
			  }
			}
			if(newFormattedType.hasDifferentFormats(format)) {
				if(newFormattedType.isPublisher()) {						const Channel newChannel = Channel(newFormattedType, format);
					attrTransChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, newNodeName, nodeName));
				} else {
					const Channel newChannel = Channel(format, newFormattedType);
					attrTransChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, nodeName, newNodeName));
		  	}
			}
			if(newFormattedType.hasDifferentTypes(format)) {
				// handle different types
			}
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
