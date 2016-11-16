#include <aseia_car_sim/RegisterCar.h>
#include <aseia_car_sim/UpdateCar.h>

#include <ros/ros.h>

#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosAddStatusbarMessage.h>
#include <vrep_common/simRosSynchronous.h>
#include <vrep_common/simRosSynchronousTrigger.h>

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
      ros::ServiceClient mStepSrv, mStartSrv, mStopSrv;
      ros::ServiceServer mRegSrv;

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

      enum class Trigger {
        start,
        stop,
        step
      };

      bool trigger(Trigger t) {
        simRosSynchronousTrigger step;
        simRosStartSimulation    start;
        simRosStopSimulation     stop;
        switch(t) {
          case(Trigger::step) : ROS_DEBUG_STREAM("Call Step Service");
                                if( !mStepSrv.call(step) )
                                  return false;
                                return step.response.result != -1;
          case(Trigger::start): ROS_DEBUG_STREAM("Call Start Service");
                                if( !mStartSrv.call(start) )
                                  return false;
                                return stop.response.result != -1;
          case(Trigger::stop) : ROS_DEBUG_STREAM("Call Stop Service");
                                if( !mStopSrv.call(stop) )
                                  return false;
                                return start.response.result != -1;
          default:              return false;
        }
      }

    public:
      Simulation() {
        ros::NodeHandle nh;
        ros::ServiceClient setSyncSrv = nh.serviceClient< simRosSynchronous >( "/vrep/simRosSynchronous");
        setSyncSrv.waitForExistence();
        mRegSrv   = nh.advertiseService(ros::this_node::getName()+"/registerCar", &Simulation::registerCar, this);
        mStepSrv  = nh.serviceClient< simRosSynchronousTrigger >( "/vrep/simRosSynchronousTrigger", true);
        mStartSrv = nh.serviceClient< simRosStartSimulation    >( "/vrep/simRosStartSimulation"   , true);
        mStopSrv  = nh.serviceClient< simRosStopSimulation     >( "/vrep/simRosStopSimulation"    , true);
        simRosSynchronous sync;
        sync.request.enable = 1;
        if(! setSyncSrv.call(sync)  || sync.response.result == -1)
          ROS_FATAL_STREAM("Switching VREP to synchronous mode failed!");
        if( !trigger(Trigger::start) )
          ROS_FATAL_STREAM("Starting VREP simulation failed!");
        ROS_INFO_STREAM("Simulation started");
      }

      void stop() {
        ROS_INFO_STREAM("Simulation ends");
        if( !trigger(Trigger::stop) )
          ROS_FATAL_STREAM("Stopping VREP simulation failed!");
        ros::spinOnce();
      }

      ~Simulation() { stop(); }

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
        if( !trigger(Trigger::step) )
          ROS_FATAL_STREAM("Advancing Simulation failed!");
      }
  };
}

using namespace car;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulation");
  ros::NodeHandle nh;
  nh.setParam("simName", ros::this_node::getName());
  Simulation sim;
  while(ros::ok()) {
    sim.update();
    ros::spinOnce();
  }
  return 0;
}
