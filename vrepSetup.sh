#/bin/bash

if [[ -z $1 ]] ; then
  echo "usage: $0 <path to vrep>"
  exit -1
fi

if [[ ! -e $1 ]] ; then
  echo "Path to vrep invalid"
  exit -1
fi

exec ln -s $1/programming/ros_packages/vrep_common src/ & ln -s $1/programming/ros_packages/vrep_plugin src/
