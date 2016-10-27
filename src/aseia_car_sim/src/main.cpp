#include "Simulation.h"

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <string>

#include <signal.h>

using namespace car;
using namespace std;
using namespace boost;

int main(int argc, char** argv) {
  ros::init(argc, argv, "CarSim");
  ros::NodeHandle nh("~");
  string ctrlNameParam;
  int carNum = 0;
  nh.getParam("controllers", ctrlNameParam);
  nh.getParam("carNum", carNum);
  Simulation::CtrlNames ctrlNames;
  split(ctrlNames, ctrlNameParam, is_any_of(" ,"), token_compress_on);
  Simulation sim(carNum, ctrlNames);
  while(ros::ok()){
    ros::spinOnce();
    sim.run();
  }
}
