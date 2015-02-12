#include <ros/ros.h>
#include <ros/console.h>

#include <EventType.h>
#include <FormatID.h>
#include <IO.h>

#include <aseia_msgs/EventType.h>
#include <aseia_msgs/SensorEvent.h>
#include <aseia_msgs/Channels.h>

#include <sstream>
#include <string>
#include <list>
#include <map>
#include <tuple>

using namespace std;
using FormatMap = map<FormatID, EventType>;
using TopicMap = map<std::string, FormatMap>;
using NodeMap  = map<std::pair<std::string, FormatID>, std::string>;

TopicMap subs;
TopicMap pubs;
NodeMap  nodes;
ros::Publisher channelPub;
aseia_msgs::Channels channelsMsg;

struct Forwarder{
  private:
    ros::Subscriber subscriber;
    ros::Publisher  publisher;

    void forward(const aseia_msgs::SensorEvent::ConstPtr& msg){
      publisher.publish(*msg);
    }

  public:
    Forwarder(const std::string& pubTopic, const std::string& subTopic) 
      : subscriber(ros::NodeHandle().subscribe(pubTopic, 10, &Forwarder::forward, this)),
        publisher(ros::NodeHandle().advertise<aseia_msgs::SensorEvent>(subTopic, 10))
    {
      aseia_msgs::Channel newChannel;
      newChannel.inTopic = pubTopic;
      newChannel.outTopic = subTopic;
      newChannel.inNode = nodes[pubTopic];
      newChannel.outNode = nodes[subTopic];
      channelsMsg.channels.push_back(newChannel);
      channelPub.publish(channelsMsg);
    }
};

using ForwardKey = tuple<string, FormatID, FormatID>;
using ForwardMap = map<ForwardKey, Forwarder>;

ForwardMap forwarders;

void handleNewPublisher(const std::string& topic, FormatMap::iterator newPub){
  ROS_INFO_STREAM("Inserting new Publisher: [" << topic << ", " << newPub->first << "]:\n" << newPub->second);
  auto subListIter = subs.find(topic);
  if(subListIter != subs.end())
    for(const auto& sub : subListIter->second)
      if(sub.second == newPub->second){
        ostringstream pubFormat, subFormat;
        pubFormat << newPub->first;
        subFormat << sub.first;
        forwarders.emplace(piecewise_construct, make_tuple(topic, newPub->first, sub.first), 
                           make_tuple(topic + "/" + pubFormat.str(), topic + "/" + subFormat.str()));
      }/*else
        call conversion function
      */
}

void handleNewSubscriber(const std::string& topic, FormatMap::iterator newSub){
  ROS_INFO_STREAM("Inserting new Subscriber: [" << topic << ", " << newSub->first << "]:\n" << newSub->second);
  auto pubListIter = pubs.find(topic);
  if(pubListIter != pubs.end())
    for(const auto& pub : pubListIter->second)
      if(pub.second == newSub->second){
        ostringstream pubFormat, subFormat;
        pubFormat << pub.first;
        subFormat << newSub->first;
        forwarders.emplace(piecewise_construct, make_tuple(topic, pub.first, newSub->first), 
                           make_tuple(topic + "/" + pubFormat.str(), topic + "/" + subFormat.str()));
      }/*else
        call conversion function
      */
}

void handleEventType(const ros::MessageEvent<aseia_msgs::EventType const>& metaData) {

  const aseia_msgs::EventType::ConstPtr& msg = metaData.getMessage();
  EventType type;


  ROS_INFO_STREAM("New node registered: " << metaData.getPublisherName());

  DeSerializer<decltype(msg->type.begin())> dType(msg->type.begin(), msg->type.end());
  dType >> type;
    
  DeSerializer<uint8_t*> dFormat((uint8_t*)&msg->format, (uint8_t*)&msg->format+sizeof(uint32_t));
  dFormat >> format;

  nodes[std::pair<std::string, FormatID>(msg->topic, format)] = metaData.getPublisherName();

  if(format.direction() == FormatID::Direction::publisher){
    auto topicResult   = pubs.insert(make_pair(msg->topic, FormatMap()));
    FormatMap& map     = topicResult.first->second;
    const string topic = topicResult.first->first;
    auto result        = map.emplace(format, type);
    if(result.second) {
      handleNewPublisher(topic, result.first);
      ostringstream info;
      info << "Publisher: " << endl;
      for(const auto& pubMap : pubs) {
        info << "\t" << topic << ": ";
        for(const auto& pub : pubMap.second)
          info << pub.first << " ";
      }
      ROS_INFO_STREAM(info.str());
    }
  }

  if(format.direction() == FormatID::Direction::subscriber){
    auto topicResult   = subs.insert(make_pair(msg->topic, FormatMap()));
    FormatMap& map     = topicResult.first->second;
    const string topic = topicResult.first->first;
    auto result        = map.emplace(format, type);
    if(result.second) {
      handleNewSubscriber(topic, result.first);
      ostringstream info;
      info << "Subscriber: " << endl;
      for(const auto& subMap : subs) {
        info << "\t" << topic << ": ";
        for(const auto& sub : subMap.second)
          info << sub.first << " ";
      }
      ROS_INFO_STREAM(info.str());
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaEventCoupler");
  ros::NodeHandle n;
  ros::Subscriber typeSub  = n.subscribe("/sensors/management", 10, handleEventType);
  channelPub = n.advertise<aseia_msgs::Channels>("/sensors/channels", 10, true);
  channelPub.publish(channelsMsg);
  while(ros::ok())
    ros::spin();
  return 0;
};
