cmake_minimum_required(VERSION 2.8.3)
project(mcl3d)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
  pcl_conversions 
  pcl_ros
  octomap_ros
  nav_msgs
)

# GSL package
find_package( PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl )

## System dependencies are found with CMake's conventions
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
#  INCLUDE_DIRS include
#  LIBRARIES stereo_processing
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gsl_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(mcl3d_node src/mcl3d_node.cpp)
add_executable(grid3d_node src/grid3d_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(mcl3d_node ${catkin_EXPORTED_TARGETS})
add_dependencies(grid3d_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(mcl3d_node
   ${catkin_LIBRARIES}
   ${gsl_LIBRARIES}
)
target_link_libraries(grid3d_node
   ${catkin_LIBRARIES}
   ${gsl_LIBRARIES}
)
