#include <IO.h>
#include <EventType.h>
#include <MetaFilter.h>
#include <DeSerializer.h>
#include <aseia_base/EventType.h>
//#include <aseia_base/Event.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/console.h>

int main(int argc, char** argv) {
  if(argc<2) {
    std::cerr << "Usage " << argv[0] << " <bagfile>" << std::endl;
    return -1;
  }
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);
  std::vector<std::string> topics = {"/sensors/management"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::ostringstream os;
  for(const rosbag::MessageInstance& m: view) {
    const aseia_base::EventType::ConstPtr& mPtr = m.instantiate<aseia_base::EventType>();
    EventType eT;
    MetaFilter filter;
    DeSerializer<decltype(mPtr->data.cbegin())> d(mPtr->data.cbegin(), mPtr->data.cend());
    d >> eT;
    if(mPtr->filter.size()) {
      d = DeSerializer<decltype(mPtr->filter.cbegin())>(mPtr->filter.cbegin(), mPtr->filter.cend());
      d >> filter;
    }
    os << "Event type: " << eT << (mPtr->type==mPtr->PUBLISHER?" published":" subscribed") << " on topic " << mPtr->topic << " by node " << mPtr->id;
    if(filter.expressions().empty())
      os << " with filter " << filter;
    ROS_INFO_STREAM(os.str());
  }
  return 0;
}
