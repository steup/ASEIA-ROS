cmake_minimum_required(VERSION 2.8.3)
project(aseia_examples)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS aseia_base)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_definitions("-std=c++11")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   EventType.msg
#   SensorEvent.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#  generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

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
	#INCLUDE_DIRS include
#LIBRARIES ${aseia_LDFLAGS}
#   CATKIN_DEPENDS genmsg
#DEPENDS aseia
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(aseia
#   src/${PROJECT_NAME}/aseia.cpp
# )


## Declare a cpp executable
add_executable(fittingPub src/fittingPub.cpp)
add_executable(diffFormatPub src/diffFormatPub.cpp)
add_executable(anglePub src/anglePub.cpp)
add_executable(anglePub2 src/anglePub2.cpp)
add_executable(anglePub3 src/anglePub3.cpp)
add_executable(positionPub src/positionPub.cpp)
add_executable(distancePub src/distancePub.cpp)
add_executable(mapMerger src/mapMerger.cpp)

add_executable(exampleSub src/exampleSub.cpp)
add_executable(angleSub src/angleSub.cpp)
add_executable(doorGridSub src/doorGridSub.cpp)
add_executable(robotSub src/robotSub.cpp)

add_dependencies(fittingPub aseia_base_generate_messages_cpp)
add_dependencies(diffFormatPub aseia_base_generate_messages_cpp)
add_dependencies(anglePub aseia_base_generate_messages_cpp)
add_dependencies(anglePub2 aseia_base_generate_messages_cpp)
add_dependencies(anglePub3 aseia_base_generate_messages_cpp)
add_dependencies(positionPub aseia_base_generate_messages_cpp)
add_dependencies(distancePub aseia_base_generate_messages_cpp)

add_dependencies(exampleSub aseia_base_generate_messages_cpp)
add_dependencies(doorGridSub aseia_base_generate_messages_cpp)
add_dependencies(robotSub aseia_base_generate_messages_cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(test)

## Specify libraries to link a library or executable target against
target_link_libraries(fittingPub ${catkin_LIBRARIES})
target_link_libraries(diffFormatPub ${catkin_LIBRARIES})
target_link_libraries(anglePub ${catkin_LIBRARIES})
target_link_libraries(anglePub2 ${catkin_LIBRARIES})
target_link_libraries(anglePub3 ${catkin_LIBRARIES})
target_link_libraries(positionPub ${catkin_LIBRARIES})
target_link_libraries(distancePub ${catkin_LIBRARIES})
target_link_libraries(mapMerger ${catkin_LIBRARIES})

target_link_libraries(exampleSub ${catkin_LIBRARIES})
target_link_libraries(angleSub ${catkin_LIBRARIES})
target_link_libraries(doorGridSub ${catkin_LIBRARIES})
target_link_libraries(robotSub ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS aseia aseia_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_aseia.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
