#include <aseia_car_sim/RegisterCar.h>
#include <aseia_car_sim/UpdateCar.h>

#include <ros/ros.h>

#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosAddStatusbarMessage.h>
#include <vrep_common/simRosSynchronous.h>
#include <vrep_common/simRosSynchronousTrigger.h>
#include <vrep_common/VrepInfo.h>

#include <signal.h>

namespace car {

  using namespace std;
  using namespace vrep_common;
  using namespace aseia_car_sim;

  class CarProxy {
    private:
      UpdateCar mState;
      string mName;
      ros::ServiceClient srv;
    public:
      CarProxy() {
        mState.response.alive = false;
      }

      CarProxy(const string& name)
        : mName(name)
      {
        reset();
      }

      ~CarProxy() {
        ROS_DEBUG_STREAM("Sending kill to car " << mName);
        mState.request.command = mState.request.KILL;
        srv.call(mState);
      }

      operator bool() const { return mState.response.alive; }

      void trigger() {
        if(mState.response.alive) {
          ROS_DEBUG_STREAM("Triggering car " << mName);
          mState.request.command = mState.request.TRIGGER;
          if( !srv.call(mState) ) {
            ROS_ERROR_STREAM("Error triggering car " << mName);
            mState.response.alive = false;
          }
          ROS_DEBUG_STREAM("Triggering car " << mName << " done");
        }
        ROS_DEBUG_STREAM("Car " << mName << " is " << (mState.response.alive?"":"not") << "alive");
      }

      bool isDone() {
        if(mState.response.alive) {
          ROS_DEBUG_STREAM("Waiting for car " << mName);
          mState.request.command = mState.request.DONE;
          if( !srv.call(mState) ) {
            ROS_ERROR_STREAM("Error waiting for car " << mName);
            mState.response.alive = false;
          }
          return mState.response.done;
          ROS_DEBUG_STREAM("car " << mName << "is " << (mState.response.done?"":"not") << " done");
        }
        ROS_DEBUG_STREAM("Car " << mName << " is " << (mState.response.alive?"":"not") << "alive");
        return true;
      }

      void reset() {
        srv = ros::NodeHandle().serviceClient<UpdateCar>(mName+"/update", true);
        mState.response.alive = true;
      }
  };

  class Simulation {
    private:
      using CarMap = std::map<string, CarProxy>;
      CarMap mCars;
      ros::ServiceServer mRegSrv;
      VrepInfo mInfo;
      ros::Subscriber mInfoSub;

      void handleInfo(VrepInfo::ConstPtr info) {
        ROS_DEBUG_STREAM("Got V-Rep simulation info");
        mInfo = *info;
      }

      bool registerCar(RegisterCar::Request& req, RegisterCar::Response& res) {
          pair<CarMap::iterator, bool> iter = mCars.emplace(req.name, req.name);
          res.result = false;
          if( iter.second ) {
            res.result = true;
            ROS_INFO_STREAM("Registered new car " << req.name);
          }
          if( !iter.second &&  !iter.first->second ) {
            iter.first->second.reset();
            res.result = true;
            ROS_INFO_STREAM("Reactivated car " << req.name);
          }
          return true;
      }

      void sync() {
        simRosSynchronous arg;
        arg.request.enable = 1;
        ros::ServiceClient srv = ros::NodeHandle().serviceClient< decltype(arg) >( "/vrep/simRosSynchronous");
        srv.waitForExistence();
        if (!srv.call(arg) || arg.response.result == -1) {
          ROS_FATAL_STREAM("Cannot enable synchronous mode of V-REP");
          ros::shutdown();
        }
      }

      void start() {
        simRosStartSimulation arg;
        ros::ServiceClient srv = ros::NodeHandle().serviceClient< decltype(arg) >( "/vrep/simRosStartSimulation");
        srv.waitForExistence();
        if (!srv.call(arg) || arg.response.result == -1) {
          ROS_FATAL_STREAM("Cannot start simulation");
          ros::shutdown();
        }
        ROS_INFO_STREAM("Simulation started");
      }

      void stop() {
        ROS_INFO_STREAM("Simulation ends");
        simRosStopSimulation arg;
        ros::ServiceClient srv = ros::NodeHandle().serviceClient< decltype(arg) >( "/vrep/simRosStopSimulation");
        srv.waitForExistence();
        if (!srv.call(arg) || arg.response.result == -1) {
          ROS_FATAL_STREAM("Cannot stop simulation");
          ros::shutdown();
        }

      }

      bool step() {
        ROS_DEBUG_STREAM("Advancing simulation");
        ros::NodeHandle nh;
        simRosSynchronousTrigger arg;
        static ros::ServiceClient srv = nh.serviceClient< decltype(arg) >( "/vrep/simRosSynchronousTrigger", true);
        if (!srv.call(arg))
          if(srv.exists())
            srv = nh.serviceClient< decltype(arg) >( "/vrep/simRosSynchronousTrigger", true);
          else {
            ROS_FATAL_STREAM("V-REP is gone! Stopping simulation!");
            ros::shutdown();
          }  
        else
          if(arg.response.result == -1)
            ROS_ERROR_STREAM("Cannot advance simulation");
          else
            return true;
        return false;
      }

    public:
      Simulation()
        : mRegSrv(ros::NodeHandle().advertiseService(ros::this_node::getName()+"/registerCar", &Simulation::registerCar, this)),
          mInfoSub(ros::NodeHandle().subscribe("/vrep/info", 1, &Simulation::handleInfo, this))
      {
        sync();
        start();
      }

      ~Simulation() { 
        cerr << "Ending Simulation" << endl;
        stop();
        while(step() && mInfo.simulatorState.data)
          ros::spinOnce();
      }

      void update() {
        ros::NodeHandle nh;
        for(CarMap::value_type& car : mCars)
          if(car.second)
            car.second.trigger();
        bool done;
        do {
          done = true;
          for(CarMap::value_type& car : mCars)
            if(car.second)
              done &=car.second.isDone();
        }while(!done);
        step();
      }
  };
}

using namespace car;

Simulation* simPtr;

void quit(int signal) {
    if(simPtr) {
      delete  simPtr;
      simPtr = nullptr;
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulation", ros::init_options::NoSigintHandler);
  signal(SIGINT, quit);
  ros::NodeHandle nh;
  nh.setParam("simName", ros::this_node::getName());
  simPtr=new Simulation();
  while(ros::ok() && simPtr) {
    simPtr->update();
    ros::spinOnce();
  }
  cerr << "Sim Program  ends" <<  endl;
  return 0;
}
