#include "Simulation.h"

#include <ros/ros.h>

#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosEnablePublisher.h>
#include <v_repConst.h>


namespace car {

  using namespace std;
  using namespace vrep_common;

  Simulation::Simulation(size_t carNum, const CtrlNames& ctrlNames) {
    ros::NodeHandle nh;
    {
      simRosStartSimulation srv;
      ros::ServiceClient client;
      do {
        client = nh.serviceClient<simRosStartSimulation>("/vrep/simRosStartSimulation");
      }while(!client.exists());
      client.call(srv);
    }
    int handle;
    {
      simRosGetObjectHandle srv;
      srv.request.objectName = "Car";
      nh.serviceClient<simRosStartSimulation>("/vrep/simRosStartSimulation").call(srv);
      handle = srv.response.handle;
    }
    simRosEnablePublisher srv;
    srv.request.topicName="/car/0";
    srv.request.queueSize=1;
    srv.request.streamCmd = simros_strmcmd_get_object_pose;
    srv.request.auxInt1 = handle;
    srv.request.auxInt2 = -1;
    srv.request.auxString = "/car/0";
    nh.serviceClient<simRosEnablePublisher>("/vrep/simRosEnablePublisher").call(srv);
  }

  void Simulation::run() {

  }

}
