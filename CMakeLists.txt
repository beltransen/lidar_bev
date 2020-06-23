cmake_minimum_required(VERSION 2.8.3)
project(lidar_bev)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
## Enable compiler optimizations
add_definitions(-O3)
add_definitions(-ffast-math)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  rosbag
  roscpp
  sensor_msgs
  tf
  tf2_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#  INCLUDE_DIRS include
#  LIBRARIES lidar_bev
#  CATKIN_DEPENDS cv_bridge image_transport pcl_ros rosbag roscpp sensor_msgs tf tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

## Declare a C++ library
add_library(bev
  src/${PROJECT_NAME}/cloud_filter.cpp
)
target_link_libraries(bev
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(offline_bev src/offline_bev.cpp)
add_executable(lidar_bev src/lidar_bev.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(offline_bev
  bev
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(lidar_bev
  bev
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
