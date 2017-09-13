#include <IO.h>
#include <EventType.h>
#include <MetaEvent.h>
#include <DeSerializer.h>
#include <ID.h>
#include <aseia_base/EventType.h>
#include <aseia_base/SensorEvent.h>
#include <aseia_car_sim/Latency.h>
#include <vrep_common/VrepInfo.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

using namespace std;
using namespace ::id::attribute;
namespace fs = boost::filesystem;

int main(int argc, char** argv) {
  if(argc<2) {
    cerr << "Usage " << argv[0] << " <bagfile>" << endl;
    return -1;
  }
  rosbag::Bag bag;
  string fileName=argv[1];
  bag.open(fileName, rosbag::bagmode::Read);
  rosbag::View vrepInfo(bag, rosbag::TopicQuery("/vrep/info"));
  {
    fs::path topicDir = fs::path(fileName).stem();
    if(!fs::exists(topicDir))
      fs::create_directories(topicDir);
    fs::ofstream file((topicDir/"vrep").replace_extension(".csv"), ios_base::out);
    if(!file.is_open())
      ROS_ERROR_STREAM("Cannot open output file for writing ");
    file << "#rostime sec in s, rostime nsec in ns, simtime in ms" << endl;
    float lastTime = -1;
    for(const rosbag::MessageInstance& m: vrepInfo) {
      const vrep_common::VrepInfo::ConstPtr& mPtr = m.instantiate<vrep_common::VrepInfo>();
      if(mPtr->simulationTime.data > lastTime) {
        file << mPtr->headerInfo.stamp.sec << ", " << mPtr->headerInfo.stamp.nsec << ", " << (uint32_t)(1000*mPtr->simulationTime.data) << endl;
        lastTime = mPtr->simulationTime.data;
      }
    }
    file.close();
  }
  rosbag::View view(bag, rosbag::TopicQuery("/sensors/management"));
  map<string, EventType> types;
  for(const rosbag::MessageInstance& m: view) {
    const aseia_base::EventType::ConstPtr& mPtr = m.instantiate<aseia_base::EventType>();
    EventType eT;
    DeSerializer<decltype(mPtr->data.cbegin())> d(mPtr->data.cbegin(), mPtr->data.cend());
    d >> eT;
    if(mPtr->type==mPtr->PUBLISHER)
      types.emplace("/sensors/"+to_string(EventID(eT))+"/"+to_string(FormatID(eT)), eT);
  }
  for(const auto& elem: types) {
    cout << elem.first << ":" << endl << elem.second << endl;

    fs::path topicDir = fs::path(fileName).stem()/to_string(EventID(elem.second));
    if(!fs::exists(topicDir))
      fs::create_directories(topicDir);
    fs::ofstream file((topicDir/to_string(FormatID(elem.second))).replace_extension(".csv"), ios_base::out);
    if(!file.is_open())
      ROS_ERROR_STREAM("Cannot open output file for writing");
    file << "#";
    size_t i=0;
    for(const AttributeType& aT: elem.second) {
      ValueType vT = aT.value();
      string attrName = id::attribute::name(aT.id());
      ssize_t endName = attrName.find_first_of(' ');
      attrName = attrName.substr(0, endName);
      if(attrName=="Unspecified")
        attrName=to_string(aT.id());
      if(vT.rows()==0 || vT.cols()==0)
        continue;
      if(vT.rows()>1 || vT.cols()>1) {
        for(size_t r=0;r<vT.rows();r++)
          for(size_t c=0;c<vT.cols();c++) {
            file << attrName << "_" << r << "_" << c;
            if(aT.unit() != Dimensionless())
              file << " in " << aT.unit();
            if(vT.hasUncertainty())
              file << ", " << attrName << "_" << r << "_" << c << " uncertainty";
            if(c+1 != vT.cols() || r+1 != vT.rows())
              file << ", ";
          }
      } else {
        file << attrName;
        if(aT.unit() != Dimensionless())
          file << " in " << aT.unit();
        if(vT.hasUncertainty())
          file << ", " << attrName << " uncertainty";
      }
      if(i++ + 1 != elem.second.length())
        file << ", ";
    }
    file << endl;
    rosbag::View subView(bag, rosbag::TopicQuery(elem.first));
    MetaEvent e(elem.second);
    for(const rosbag::MessageInstance& m: subView) {
      const aseia_base::SensorEvent::ConstPtr& mPtr = m.instantiate<aseia_base::SensorEvent>();
      if(!mPtr)continue;
      DeSerializer<decltype(mPtr->event.cbegin())> d(mPtr->event.cbegin(), mPtr->event.cend());
      d >> e;
      i=0;
      for(const AttributeType& aT: elem.second) {
        MetaAttribute a = e[aT.id()];
        ValueType vT = aT.value();
        vT.typeId(id::type::Double());
        a.value() = a.value().cast(vT);
        a/=a.scale();
        const MetaValue& v=a.value();
        for(size_t r=0;r<vT.rows();r++)
          for(size_t c=0;c<vT.cols();c++) {
            file << v.get(r, c).value();
            if(vT.hasUncertainty())
              file << ", " << v.get(r, c).uncertainty();
            if(c+1 != v.cols() || r+1 != v.rows())
              file << ", ";
          }
        if(i++ + 1 != elem.second.length())
          file << ", ";
      }
      file << endl;
    }
    file.close();
  }
  rosbag::View recvInfo(bag, rosbag::TopicQuery("/latency"));
  {
    fs::path topicDir = fs::path(fileName).stem();
    if(!fs::exists(topicDir))
      fs::create_directories(topicDir);
    fs::ofstream file((topicDir/"latency").replace_extension(".csv"), ios_base::out);
    if(!file.is_open())
      ROS_ERROR_STREAM("Cannot open output file for writing ");
    file << "#topic, send time in ms, recv time in ms, object, publisher, car index" << endl;
    for(const rosbag::MessageInstance& m: recvInfo) {
      const aseia_car_sim::Latency::ConstPtr& mPtr = m.instantiate<aseia_car_sim::Latency>();
      if(!mPtr)continue;
      file << mPtr->topic << ", " << mPtr->send << ", " << mPtr->recv << ", " << mPtr->object << ", " << mPtr->publisher << ", " << mPtr->car;
    }
    file.close();
  }

  return 0;
}
