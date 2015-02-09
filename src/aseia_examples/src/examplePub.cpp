#include <ros/ros.h>
#include <ros/console.h>

#include <SensorEventPublisher.h>
#include <BaseEvent.h>
#include <ID.h>

#include <signal.h>

#include <chrono>
#include <thread>
#include <atomic>

std::atomic<bool> running;

void finish(int sig){
  ros::shutdown();
}

void run(){
    BaseEvent<> e;
    SensorEventPublisher<decltype(e)> pub("test", 0);
    e.attribute(id::attribute::Position())    = { {0,0}, {0,0} };
    e.attribute(id::attribute::Time())        = { {(unsigned int)ros::Time::now().toSec(), 1} };
    e.attribute(id::attribute::PublisherID()) = { {0} };
    e.attribute(id::attribute::Validity())    = { {1.0} };
    do{
      ROS_INFO_STREAM("publish: " << e);
      pub.publish(e);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }while(running);
    ROS_INFO_STREAM("finished");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaTestPub");
  signal(SIGINT, finish);

  ROS_INFO_STREAM("started");

  running=true;

  std::thread t(run);

  while(ros::ok())
    ros::spin();
    
  running=false;
  t.join();
  return 0;
}
