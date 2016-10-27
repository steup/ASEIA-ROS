#include "Simulation.h"

#include <ros/ros.h>

#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosAddStatusbarMessage.h>
#include <vrep_common/simRosSynchronous.h>
#include <vrep_common/simRosSynchronousTrigger.h>


namespace car {

  using namespace std;
  using namespace vrep_common;

  Simulation::Simulation(size_t carNum, const CtrlNames& ctrlNames) {
    ros::NodeHandle nh;
    ros::ServiceClient client;
    do {
      client = nh.serviceClient< simRosSynchronous >( "/vrep/simRosSynchronous");
    }while(!client.exists());
    simRosSynchronous sync;
    sync.request.enable = 1;
    client.call(sync);
    if(sync.response.result == -1)
      ROS_FATAL_STREAM("Switching VREP to synchronous mode failed!");
    for( size_t i = 0; i < carNum; i++ ) {
      mCars.emplace_back(i);
      for(const string& ctrl : ctrlNames) {
        ControllerPtr&& ctrlPtr = ctrlFactory(ctrl);
        if( ctrlPtr )
          mCars.back().addController(std::move(ctrlPtr));
      }
    }
    simRosStartSimulation simStart;
    nh.serviceClient< simRosStartSimulation >( "/vrep/simRosStartSimulation" ).call(simStart);
    if(simStart.response.result == -1)
      ROS_FATAL_STREAM("Starting VREP simulation failed!");
    ROS_INFO_STREAM("Simulation started");
  }

  Simulation::~Simulation() {
    ros::NodeHandle nh;
    ros::ServiceClient client;
    do {
      client = nh.serviceClient< simRosStopSimulation >( "/vrep/simRosStopSimulation");
    }while(!client.exists());
    simRosStopSimulation simStop;
    client.call(simStop);
    ROS_INFO_STREAM("Simulation ended");
    ros::spinOnce();
  }

  void Simulation::run() {
    ros::NodeHandle nh;
    simRosSynchronousTrigger trigger;
    for(Car& car : mCars)
      car.update();
    nh.serviceClient< simRosSynchronousTrigger >( "/vrep/simRosSynchronousTrigger" ).call(trigger);
    if(trigger.response.result == -1)
      ROS_FATAL_STREAM("Triggering VREP failed!");
    ROS_DEBUG_STREAM("Update!!!");
  }

}
