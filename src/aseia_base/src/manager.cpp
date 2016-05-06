#include <ros/ros.h>
#include <ros/console.h>

#include <BaseEvent.h>
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

#include <cmath>

using namespace std;
using namespace id::attribute;

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

//--------------------------------------------------------------------------------------------------
// handle angle events

void bresenham(int16_t x1, int16_t y1, int16_t x2, int16_t y2, Value<uint8_t, 24, 24, false> &grid){
      int16_t delta_x(x2-x1);
      signed char const ix((delta_x > 0) - (delta_x < 0));
      delta_x = 2 * std::abs(delta_x);

      int16_t delta_y(y2-y1);
      signed char const iy((delta_y > 0) - (delta_y < 0));
      delta_y = 2 * std::abs(delta_y);

      //drawPixel(x1, y1);
      grid(y1, x1) = 77;

      if(delta_x >= delta_y){
        int16_t error(delta_y - (delta_x/2));

        while(x1 != x2){
          if((error >= 0) && (error || (ix > 0))){
            error -= delta_x;
            y1 += iy;
          }

          error += delta_y;
          x1 += ix;

          //drawPixel(x1, y1);
          grid(y1, x1) = 77;
        }
      }
      else{
        int16_t error(delta_x - (delta_y/2));

        while(y1 != y2){
          if((error >= 0) && (error || (iy > 0))){
            error -= delta_y;
            x1 += ix;
          }

          error += delta_x;
          y1 += iy;

          //drawPixel(x1, y1);
          grid(y1, x1) = 77;
        }
      }
    }

// door is 1.2m long, cell size is 0.05m
// FIXME: * at the moment this expects an AngleEvent and directly transforms it into a door grid
//        * change to DetectionEvent with ObjectType "Door"
struct DoorToGridTransformer: public EventHandler{
  //private:
    Value<uint8_t, 24, 24, false> grid;

    void drawPixel(int16_t x, int16_t y){
      grid(y,x) = 77;
    }  

    struct AngleEventConfig : public BaseConfig
    {
      using PositionValueType    = Value<int16_t, 2>;
      using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
      using PositionScale        = std::ratio<1, 100>;
    };

    using AngleAttribute = Attribute<Angle, Value<int16_t, 1>, Radian,  std::ratio<1, 100>>;
    using AngleEvent = BaseEvent<AngleEventConfig>::append<AngleAttribute>::type;

    struct DoorGridEventConfig : public BaseConfig
    {
      using PositionValueType    = Value<int16_t, 2>;
      using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
      using PositionScale        = std::ratio<1, 100>;
    };

    using DoorGridAttribute = Attribute<OccupancyGrid, Value<uint8_t, 24, 24, false>, Dimensionless>;
    using DoorGridEvent = BaseEvent<DoorGridEventConfig>::append<DoorGridAttribute>::type;

    virtual void handle(const aseia_base::SensorEvent::ConstPtr& msg){
      // unpack message into event
      AngleEvent ae;
      DeSerializer<decltype(msg->event.begin())> d(msg->event.begin(), msg->event.end());
      d >> ae;

      uint16_t doorLength = 23;
      int16_t  startX     = 0;
      int16_t  startY     = 0;

      int16_t angle = ae.attribute(Angle()).value().value().value();
      int16_t endX  = startX + doorLength * cos(angle);
      int16_t endY  = startY + doorLength * sin(angle);

      bresenham(startX, startY, endX, endY, grid);

      // build and publish a new message containing the Occupancy Grid
      aseia_base::SensorEvent newMsg;
      DoorGridEvent dge;

      dge.attribute(Position()).value()      = { {{-148, 0}}, {{195, 0}} };
      dge.attribute(PublisherID()).value()   = { {{(unsigned long)std::time(nullptr),1}} };
      dge.attribute(Time()).value()          = { {{1338}} };
      dge.attribute(OccupancyGrid()).value() = grid;

      //ROS_INFO_STREAM("grid: " << dge);

      newMsg.event.resize(dge.size());
      Serializer<decltype(newMsg.event.begin())> s(newMsg.event.begin());
      s << dge;

      mPublisher.publish(newMsg);
    }

  public:
    DoorToGridTransformer(const Channel& channel, const NodeName& pubName, const NodeName& subName) 
      : EventHandler(channel, pubName, subName)
    {}
};


//--------------------------------------------------------------------------------------------------
// handle distance events

struct DistanceToAngleTransformer: public EventHandler{

    struct DistanceEventConfig : public BaseConfig
    {
      using PositionValueType    = Value<int16_t, 2>;
      using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
      using PositionScale        = std::ratio<1, 100>;
    };

    using DistanceAttribute = Attribute<Distance, Value<int16_t, 1>, Meter, std::ratio<1,100>>;
    using DistanceEvent = BaseEvent<DistanceEventConfig>::append<DistanceAttribute>::type;

    struct AngleEventConfig : public BaseConfig
    {
      using PositionValueType    = Value<int16_t, 2>;
      using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
      using PositionScale        = std::ratio<1, 100>;
    };

    using AngleAttribute = Attribute<Angle, Value<int16_t, 1>, Radian,  std::ratio<1, 100>>;
    using AngleEvent = BaseEvent<AngleEventConfig>::append<AngleAttribute>::type;


    virtual void handle(const aseia_base::SensorEvent::ConstPtr& msg){
      // unpack message into event
      DistanceEvent de;
      DeSerializer<decltype(msg->event.begin())> d(msg->event.begin(), msg->event.end());
      d >> de;

      uint16_t distance = de.attribute(Distance()).value().value().value();
      
      int16_t sensorPosX = de.attribute(Position()).value()(0);
      int16_t sensorPosY = de.attribute(Position()).value()(1);

      int16_t doorHingeX = sensorPosX + 1;  // OC - x
      int16_t doorHingeY = sensorPosY - 1;

      int16_t doorX = sensorPosX - distance;
      int16_t doorY = sensorPosY;

      int16_t hingeToDoorX = doorX - doorHingeX;  // CA - x
      int16_t hingeToDoorY = doorY - doorHingeY;  // CA - y

      float numerator = doorHingeX * hingeToDoorX;
      float denominator = sqrt(doorHingeX * doorHingeX ) * 
                            sqrt(hingeToDoorX * hingeToDoorX + hingeToDoorY * hingeToDoorY);

      float alpha = acos(numerator / denominator);


      // build and publish a new message containing the Occupancy Grid
      aseia_base::SensorEvent newMsg;
      AngleEvent ae;

      ae.attribute(Position()).value()      = { {{3, 0}}, {{3, 0}} };
      ae.attribute(PublisherID()).value()   = { {{(unsigned long)std::time(nullptr),1}} };
      ae.attribute(Time()).value()          = { {{1338}} };
      ae.attribute(Angle()).value()          = { {{alpha * 180 / M_PI}} };


      //ROS_INFO_STREAM("grid: " << ae);

      newMsg.event.resize(ae.size());
      Serializer<decltype(newMsg.event.begin())> s(newMsg.event.begin());
      s << ae;

      mPublisher.publish(newMsg);
    }

  public:
    DistanceToAngleTransformer(const Channel& channel, const NodeName& pubName, const NodeName& subName) 
      : EventHandler(channel, pubName, subName)
    {}
};


//--------------------------------------------------------------------------------------------------
// handle position events

void drawCircle(uint16_t centerX, uint16_t centerY, uint16_t r, Value<uint8_t, 14, 14, false> &grid){
  for(uint16_t x = 0; x < 14; x++){
    for(uint16_t y = 0; y < 14; y++){
      uint16_t tmpX = x + centerX - 6;
      uint16_t tmpY = y + centerY - 6;

      uint16_t dx = centerX - tmpX;
      uint16_t dy = centerY - tmpY;

      if((dx*dx + dy*dy) <= r*r){
        grid(y,x) = 77;
      }
    }
  }
}


struct PositionToGridTransformer: public EventHandler{
  //private:
    Value<uint8_t, 14, 14, false> grid;

    struct PositionEventConfig : public BaseConfig
    {
      using PositionValueType    = Value<int16_t, 2>;
      using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
      using PositionScale        = std::ratio<1, 100>;
    };

    using PositionEvent = BaseEvent<PositionEventConfig>;

    struct RobotGridEventConfig : public BaseConfig
    {
      using PositionValueType    = Value<int16_t, 2>;
      using PublisherIDValueType = Value<uint16_t, 1, 1, false>;
      using PositionScale        = std::ratio<1, 100>;
    };

    using RobotGridAttribute = Attribute<OccupancyGrid, Value<uint8_t, 14, 14, false>, Dimensionless>;
    using RobotGridEvent = BaseEvent<RobotGridEventConfig>::append<RobotGridAttribute>::type;

    virtual void handle(const aseia_base::SensorEvent::ConstPtr& msg){
      // unpack message into event
      PositionEvent pe;
      DeSerializer<decltype(msg->event.begin())> d(msg->event.begin(), msg->event.end());
      d >> pe;

      // FIXME: static radius for now
      uint16_t radius  = 6;

      int16_t centerX = pe.attribute(Position()).value()(0);
      int16_t centerY = pe.attribute(Position()).value()(1);

      drawCircle(centerX, centerY, radius, grid);

      // build and publish a new message containing the Occupancy Grid
      aseia_base::SensorEvent newMsg;
      RobotGridEvent rge;

      rge.attribute(Position()).value()      = { {{3, 0}}, {{3, 0}} };
      rge.attribute(PublisherID()).value()   = { {{(unsigned long)std::time(nullptr),1}} };
      rge.attribute(Time()).value()          = { {{1338}} };
      rge.attribute(OccupancyGrid()).value() = grid;

      //ROS_INFO_STREAM("grid: " << rge);

      newMsg.event.resize(rge.size());
      Serializer<decltype(newMsg.event.begin())> s(newMsg.event.begin());
      s << rge;

      mPublisher.publish(newMsg);
    }

  public:
    PositionToGridTransformer(const Channel& channel, const NodeName& pubName, const NodeName& subName) 
      : EventHandler(channel, pubName, subName)
    {}
};

using SimpleChannelMap = map<Channel, Forwarder>;
using FormatTransformChannelMap = map<Channel, AttributeTransformer>;

using DoorToGridMap = map<Channel, DoorToGridTransformer>;
DoorToGridMap doorChannels;

using PositionToGridMap = map<Channel, PositionToGridTransformer>;
PositionToGridMap positionChannels;

using DistanceToAngle = map<Channel, DistanceToAngleTransformer>;
DistanceToAngle distanceChannels;
//--------------------------------------------------------------------------------------------------


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
				if(newFormattedType.isPublisher()) {
          const Channel newChannel = Channel(newFormattedType, format);
					attrTransChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, newNodeName, nodeName));
				} else {
					const Channel newChannel = Channel(format, newFormattedType);
					attrTransChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, nodeName, newNodeName));
		  	}
			}
			if(newFormattedType.hasDifferentTypes(format)) {
        if(newFormattedType.isPublisher()) {            
          if(newFormattedType.mTypeName == "/sensors/angle" && format.mTypeName == "/sensors/door"){ 
            const Channel newChannel = Channel(newFormattedType, format);
            doorChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, newNodeName, nodeName));
          }
          if(newFormattedType.mTypeName == "/sensors/position" && format.mTypeName == "/sensors/robot"){ 
            const Channel newChannel = Channel(newFormattedType, format);
            positionChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, newNodeName, nodeName));
          }
          if(newFormattedType.mTypeName == "/sensors/distance" && format.mTypeName == "/sensors/angle"){ 
            const Channel newChannel = Channel(newFormattedType, format);
            distanceChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, newNodeName, nodeName));
          }
        } else {
            if(newFormattedType.mTypeName == "/sensors/door" && format.mTypeName == "/sensors/angle"){
              const Channel newChannel = Channel(format, newFormattedType);
              doorChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, nodeName, newNodeName));
            }
            if(newFormattedType.mTypeName == "/sensors/robot" && format.mTypeName == "/sensors/position"){
              const Channel newChannel = Channel(format, newFormattedType);
              positionChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, nodeName, newNodeName));
            }
            if(newFormattedType.mTypeName == "/sensors/angle" && format.mTypeName == "/sensors/distance"){
              const Channel newChannel = Channel(format, newFormattedType);
              distanceChannels.emplace(piecewise_construct, make_tuple(newChannel), make_tuple(newChannel, nodeName, newNodeName));
            }
        }
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
