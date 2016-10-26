#include "Simulation.h"

#include <ros/ros.h>

#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosAddStatusbarMessage.h>


namespace car {

  using namespace std;
  using namespace vrep_common;

  Simulation::Simulation(size_t carNum, const CtrlNames& ctrlNames) {
    ros::NodeHandle nh;
    ros::ServiceClient client;
    do {
      client = nh.serviceClient< simRosAddStatusbarMessage >( "/vrep/simRosAddStatusbarMessage");
    }while(!client.exists());
    simRosAddStatusbarMessage msg;
    msg.request.message = "Simulation Node starting";
    client.call(msg);
    for( size_t i = 0; i < carNum; i++ ) {
      mCars.emplace_back(i);
      for(const string& ctrl : ctrlNames) {
        ControllerPtr&& ctrlPtr = ctrlFactory(ctrl);
        if( ctrlPtr )
          mCars.back().addController(std::move(ctrlPtr));
      }
    }
  }

  void Simulation::run() {
    ros::NodeHandle nh;
    simRosStartSimulation simStart;
    nh.serviceClient< simRosStartSimulation >( "/vrep/simRosStartSimulation" ).call(simStart);
  }

}
