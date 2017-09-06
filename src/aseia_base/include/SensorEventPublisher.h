#pragma once

#include <AbstractPubSub.h>

#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <aseia_base/SensorEvent.h>
#include <ID.h>
#include <Prime.h>

template<typename SensorEvent>
class SensorEventPublisher : public AbstractPubSub<SensorEvent>{
  private:
    using Base = AbstractPubSub<SensorEvent>;
    ros::Publisher mPub;
    int            mNodeID = 0;

  public:
    SensorEventPublisher(size_t size=1, bool latch=false)
      : Base(Base::Type::publisher) {
        mPub = ros::NodeHandle().advertise< aseia_base::SensorEvent >(this->topic(), size, latch);
        ros::TimerEvent time;
        if(!ros::NodeHandle("~").getParam("id", mNodeID))
          ROS_ERROR_STREAM("No \"id\" specified using 0: Events will not be used in Transformations!");
        this->publishType(time);
    }

    void publish(SensorEvent& e) {
      aseia_base::SensorEvent buffer;
      e[id::attribute::PublisherID()].value()={{{PrimeGenerator::prime(mNodeID)}}};
      buffer.event.resize(SensorEvent::size());
      Serializer<std::vector<uint8_t>::iterator> s(buffer.event.begin());
      s << e;
      mPub.publish(buffer);
    }
};
