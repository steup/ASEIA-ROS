#include "Errors.h"

#include <ros/ros.h>

using namespace aseia_car_sim;

void ErrorGenerator::dynReConfCallback(ErrorConfig& config, uint32_t level) {
  ROS_INFO_STREAM("Reconfigured sigma and confidence");
  mTimeSigma  = config.time_sigma;
  mDistSigma  = config.distance_sigma;
  mPosSigma   = config.position_sigma;
  mOriSigma   = config.orientation_sigma;
  mConfidence = config.confidence;
};

ErrorGenerator::ErrorGenerator() {
  int seed;
  ros::NodeHandle nh("~");
  if(!nh.getParam("seed", seed)){
    seed=1337;
    ROS_WARN_STREAM("Random Seed not set, using 1337");
  }
  mEngine.seed(seed);
  dynamic_reconfigure::Server<ErrorConfig>::CallbackType f;
  f = boost::bind(&ErrorGenerator::dynReConfCallback, this, _1, _2);
  mDynReConfServer.setCallback(f);
}

ErrorGenerator& ErrorGenerator::instance() {
  static ErrorGenerator sGen;
  return sGen;
}

