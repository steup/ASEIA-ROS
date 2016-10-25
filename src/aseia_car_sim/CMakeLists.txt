cmake_minimum_required(VERSION 2.8.3)
project(aseia_car_sim)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS roscpp aseia_base)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(simulation src/main.cpp src/Simulation.cpp src/Car.cpp)
target_link_libraries(simulation ${catkin_LIBRARIES})
