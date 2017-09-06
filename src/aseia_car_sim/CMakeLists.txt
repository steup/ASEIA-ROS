cmake_minimum_required(VERSION 2.8.3)
project(aseia_car_sim)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  nav_msgs
  std_msgs
  tf
  pluginlib
  dynamic_reconfigure
  aseia_base
  vrep_plugin
  vrep_common
)

add_definitions(-std=c++11 -Wall -g ${aseiaBase_DEFINITIONS})
set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} -flto=4 -fuse-linker-plugins)

add_message_files(
  FILES
  EventData.msg
)

add_service_files(
   FILES
   RegisterCar.srv
)

generate_dynamic_reconfigure_options(
  cfg/Nurbs.cfg
  cfg/Error.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(CarSimTransformations src/UTMToRoad.cpp
                                  src/VirtACC.cpp
                                  src/VirtACCKalman.cpp
                                  src/VirtSpeed.cpp)
add_dependencies(CarSimTransformations ${PROJECT_NAME}_gencfg)
add_executable(simulation src/Simulation.cpp)
add_executable(bridge src/Bridge.cpp)
add_dependencies(bridge ${PROJECT_NAME}_generate_messages_cpp)
add_executable(road src/Road.cpp)
add_executable(car src/Car.cpp src/Data.cpp src/Controller.cpp
  src/VRepWrapper.cpp src/Errors.cpp)
add_executable(extract src/extract.cpp)
target_link_libraries(simulation ${catkin_LIBRARIES})
target_link_libraries(car ${catkin_LIBRARIES})
target_link_libraries(bridge ${catkin_LIBRARIES})
target_link_libraries(road ${catkin_LIBRARIES})
target_link_libraries(CarSimTransformations ${catkin_LIBRARIES})
target_link_libraries(extract ${catkin_LIBRARIES} ASEIA)
add_dependencies(simulation ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(car ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
