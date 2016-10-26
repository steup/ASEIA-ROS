#include "Simulation.h"

#include <ros/ros.h>
#include <string>

using namespace car;

int main(int argc, char** argv) {
  ros::init(argc, argv, "CarSim");
  Simulation::CtrlNames ctrlNames;
  std::size_t carNum = 0;
  Simulation sim(carNum, ctrlNames);
  sim.run();
}
