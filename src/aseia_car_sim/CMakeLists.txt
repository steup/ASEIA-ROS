cmake_minimum_required(VERSION 2.8.3)
project(aseia_car_sim)

add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS roscpp aseia_base vrep_plugin
  vrep_common)

add_service_files(
   FILES
   RegisterCar.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(simulation src/Simulation.cpp)
add_executable(car src/Car.cpp src/Data.cpp src/Controller.cpp src/VRepWrapper.cpp)
target_link_libraries(simulation ${catkin_LIBRARIES})
target_link_libraries(car ${catkin_LIBRARIES})
add_dependencies(simulation ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(car ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
