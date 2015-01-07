cmake_minimum_required(VERSION 2.8.3)
project(vrep_plugin)

find_package(catkin REQUIRED roscpp tf)

# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include)

## Declare a cpp library
add_library(v_repExtROS
   src/vrep_plugin.cpp
   src/v_repLib.cpp
   src/ROS_server.cpp
   src/vrepSubscriber.cpp
   src/v_repLib.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(v_repExtROS
  ${catkin_LIBRARIES}
  pthread
  dl
)
