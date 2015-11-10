cmake_minimum_required(VERSION 2.8.3)
project(GPMDS)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
rosbag
nl_msgs
passive_ds_control
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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
  LIBRARIES gpmds
  CATKIN_DEPENDS nl_msgs
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_library(gpmds src/MultiGPR.cpp src/GPMDS.cpp)
# also listing includes just to have them show up in qtcreator
add_executable(expData src/expData.cpp include/GPMDS/GPMDS.h include/GPMDS/MultiGPR.h)
add_executable(exportReshapingParams src/exportReshapingParams.cpp)
target_link_libraries(expData gpmds)
target_link_libraries(exportReshapingParams gpmds ${catkin_LIBRARIES})

#option(USE_DOUBLE_PRECISION "Use double precision" on)
#if(USE_DOUBLE_PRECISION)
add_definitions(-DUSE_DOUBLE_PRECISION)
#endif()
