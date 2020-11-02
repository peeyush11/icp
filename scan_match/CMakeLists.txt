cmake_minimum_required(VERSION 2.8.3)
project(scan_match)

## Use C++11
add_definitions(-std=c++11)

## Debugging things vs Release Optimizations
##   Set flags for each mode
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
##   Default to release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
message("Build type: ${CMAKE_BUILD_TYPE}")
##   Type dependent definitions
if(${CMAKE_BUILD_TYPE} MATCHES Debug)
  add_definitions(-DDEBUG_MODE)
endif()



## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    pcl_ros
    message_filters 
    laser_geometry
    rospy
    tf_conversions
    eigen_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    sensor_msgs
    std_msgs
#  DEPENDS
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_core
   src/PCLScanRegistration.cpp
   src/EigenScanRegistration.cpp
   src/HeuristicRegistration.cpp
   src/RosUtils.cpp
   src/LeastSquareICP.cpp
   src/NormalEstimation.cpp
 )
add_library(${PROJECT_NAME}_strategy
   #src/strategy/ScanMatchingStrategy.cpp
   src/strategy/NearestNeighbor.cpp
   src/strategy/PclStrategy.cpp
   src/strategy/HeuristicStrategy.cpp
   src/strategy/LeastSquare.cpp
   )
 add_library(${PROJECT_NAME}_listener
   src/listener/EigenListener.cpp
 )

target_link_libraries(${PROJECT_NAME}_listener ${PROJECT_NAME}_core)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

## Declare cpp executables
add_executable(scan_match
  src/listener/ScanHandler.cpp
  src/listener/ScanHandlerNode.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_strategy 
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)


target_link_libraries(scan_match
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_strategy
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(
  TARGETS scan_match
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}_test test/eigen_listener_test.cpp test/LS_test.cpp test/heuristic_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test 
    ${PROJECT_NAME}_core ${PROJECT_NAME}_listener 
    ${PROJECT_NAME}_strategy 
    ${catkin_LIBRARIES})
endif()
