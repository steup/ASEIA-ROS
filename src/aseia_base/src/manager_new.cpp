#include <ros/ros.h>
#include <ros/console.h>

#include <aseia_base/SensorEvent.h>
#include <aseia_base/EventType.h>

#include <Channel.h>
#include <MetaEvent.h>
#include <MetaFactory.h>
#include <FormatID.h>
#include <EventID.h>

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
				MetaEvent  e=MetaFactory::instance().create(*eTPtr);
				DeSerializer<decltype(msgPtr->event.begin())> d(msgPtr->event.begin(), msgPtr->event.end());
				d >> e;
				handleEvent(e);
		}

		RosChannel(TransPtr&& trans) : Channel(std::move(trans)) {
			ros::NodeHandle n;
			mPub =  n.advertise<aseia_base::SensorEvent>(topic(trans->out()), 1);
			for(const EventType* in : trans->in()) {
				mSubs.push_back(n.subscribe<aseia_base::SensorEvent>(topic(*in), 1, boost::bind(&RosChannel::unpackEvent,  this,  _1, in)));
			}
		}

		virtual void publishEvent(const MetaEvent& e) const {
			aseia_base::SensorEvent sE;
			Serializer<decltype(back_inserter(sE.event))> s(back_inserter(sE.event));
			s << e;
			mPub.publish(sE);
		}
};

class ChannelManager {
	private:
		ros::Subscriber mSub;
	public:
		void handleNode(aseia_base::EventTypeConstPtr eTPtr) {
			EventType eT;
			DeSerializer<decltype(eTPtr->data.begin())> d(eTPtr->data.begin(), eTPtr->data.end());
			d >> eT;

		}
		ChannelManager() {
			ros::NodeHandle n;
			mSub = n.subscribe("/sensors/management", 1, &ChannelManager::handleNode, this);
		}
};

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaChannelManager");
  ChannelManager manager;
  while(ros::ok())
    ros::spin();
  return 0;
};
