#include <Channel.h>

#include <EventType.h>
#include <TransformationFactory.h>

#include <sstream>

Channel::Channel(const std::list<const EventType*>& in, const EventType& out, TransID& trans)
  : mOutType(out),
    mInTypes(in)
{
  ros::NodeHandle nh;
  TransformationFactory& f = TransformationFactory::instance();
  mTrans.reset(f.create(trans));
  ostringstream os;
  os << "/sensors/" << EventID(mOutType) << "/" << FormatID(mOutType);
  mPub = nh.advertise<aseia::SensorEvent>(os.str(), 1);
  for(const EventType& in : in) {
    ostringstream os;
    os << "/sensors/" << EventID(in) << "/" << FormatID(in);
    mSubs.push_back(nh.subscribe(os.str(), &Channel::handleEvent, 1));
  }
}

void Channel::handleEvent(aseia::SensorEventConstPtr& msg) {

}

const EventType& Channel::inType(std::size_t i) const {
  if(i < inputSize())
    return mInTypes[i];
  else
    return EventType();
}

std::ostream operator<<(std::ostream& o, const Channel& c) {
  return o;
}
