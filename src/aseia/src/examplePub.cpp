#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <signal.h>

#include <chrono>
#include <thread>
#include <atomic>

void finish(int sig){
  ros::shutdown();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "AseiaTestPub");
  signal(SIGINT, finish);

  ros::NodeHandle n;

  ROS_INFO_STREAM("started");

  ros::Publisher pub = n.advertise<std_msgs::String>("example", 10);
  
  bool running = true;
  std::thread t(
    [&pub, &running](){
      uint16_t count=0;
      std_msgs::String msg;
      do{
        ROS_INFO_STREAM("publish");
        msg.data = std::string("Count: ")+std::to_string(count++);
        pub.publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }while(running);
      ROS_INFO_STREAM("finished");
    }
  );

  while(ros::ok())
    ros::spin();
    
  running=false;
  t.join();
  return 0;
}
