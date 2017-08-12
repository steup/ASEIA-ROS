#include <aseia_car_sim/RegisterCar.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosAddStatusbarMessage.h>
#include <vrep_common/simRosAddStatusbarMessage.h>
#include <vrep_common/simRosSynchronous.h>
#include <vrep_common/simRosSynchronousTrigger.h>
#include <vrep_common/simRosSetBooleanParameter.h>
#include <vrep_common/VrepInfo.h>
#include "v_repConst.h"

#include <signal.h>

namespace car {

  using namespace std;
  using namespace vrep_common;
  using namespace aseia_car_sim;

  class SimHandle {
    public:
      virtual void updateCars() = 0;
  };

  class CarProxy {
    private:
      string mName;
      ros::Subscriber mSub;
      SimHandle& mSim;
      bool mDone, mAlive;

      void handleDone(const std_msgs::BoolConstPtr& alive) {
        mDone=true;
        mAlive=alive->data;
        mSim.updateCars();
      }

    public:
      CarProxy(const string& name, SimHandle& sim)
        : mName(name),
          mSim(sim)
      {
        mSub=ros::NodeHandle().subscribe(mName+"/done", 1, &CarProxy::handleDone, this);
        reset();
      }

      ~CarProxy() {
        ROS_DEBUG_STREAM("Sending kill to car " << mName);
      }

      operator bool() const { return mAlive; }

      void trigger() {
        if(mAlive) {
          ROS_DEBUG_STREAM("Triggering car " << mName);
          mDone = false;
        }
        ROS_DEBUG_STREAM("Car " << mName << " is " << (mAlive?"":"not") << "alive");
      }
      
      bool isDone() const { return mDone; }

      void reset() {
        mAlive = true;
        mDone = true;
      }
  };

  class Simulation : public SimHandle {
    private:
      using CarMap = std::map<string, CarProxy>;
      CarMap mCars;
      ros::ServiceServer mRegSrv;
      VrepInfo mInfo;
      ros::Subscriber mInfoSub;
      ros::Publisher mCarUpdatePub;
      ros::Timer mTimeout;

      void handleInfo(VrepInfo::ConstPtr info) {
        ROS_DEBUG_STREAM("Got V-Rep simulation info");
        mInfo = *info;
      }

      bool registerCar(RegisterCar::Request& req, RegisterCar::Response& res) {
          pair<CarMap::iterator, bool> iter = mCars.emplace(piecewise_construct,
                                                forward_as_tuple(req.name),
                                                forward_as_tuple(req.name, *this)
                                              );
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

      void enableThreadedRendering() {
        simRosSetBooleanParameter arg;
        arg.request.parameter = sim_boolparam_threaded_rendering_enabled;
        arg.request.parameterValue = true;
        ros::ServiceClient srv = ros::NodeHandle().serviceClient< decltype(arg) >( "/vrep/simRosSetBooleanParameter");
        srv.waitForExistence();
        if (!srv.call(arg) || arg.response.result == -1)
          ROS_ERROR_STREAM("Cannot enable Threaded Rendering");
        else
          ROS_INFO_STREAM("Threaded Rendering enabled");
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
          mInfoSub(ros::NodeHandle().subscribe("/vrep/info", 1, &Simulation::handleInfo, this)),
          mCarUpdatePub(ros::NodeHandle().advertise<std_msgs::Bool>("/cars/update", 1)),
          mTimeout(ros::NodeHandle().createTimer(ros::Duration(1), &Simulation::update, this, true))
      {
        sync();
        start();
        enableThreadedRendering();
      }

      virtual ~Simulation() {
        cerr << "Ending Simulation" << endl;
        std_msgs::Bool msg;
        msg.data=false;
        mCarUpdatePub.publish(msg);
        stop();
        do{
          ros::spinOnce();
        }while(step() && mInfo.simulatorState.data);
      }


      void update(const ros::TimerEvent& e = ros::TimerEvent()) {
        ros::NodeHandle nh;
        std_msgs::Bool msg;
        msg.data = true;
        mCarUpdatePub.publish(msg);
        mTimeout = ros::NodeHandle().createTimer(ros::Duration(1), &Simulation::update, this, true);
        for(CarMap::value_type& car : mCars)
          if(car.second)
            car.second.trigger();
      }

      virtual void updateCars() {
        bool done;
        done = true;
        for(CarMap::value_type& car : mCars)
          if(car.second)
            done &=car.second.isDone();
        if(done){
          mTimeout.stop();
          step();
          update();
        }
      }
  };
}

using namespace car;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulation");
  ros::NodeHandle nh;
  nh.setParam("simName", ros::this_node::getName());
  Simulation* simPtr = new Simulation();
  while(ros::ok()) {
    ros::spin();
  }
  delete simPtr;
  cerr << "Sim Program  ends" <<  endl;
  return 0;
}
