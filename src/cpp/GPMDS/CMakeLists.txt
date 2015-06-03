cmake_minimum_required(VERSION 2.4.6)

set(CMAKE_MODULE_PATH /usr/local/share/cmake/Modules/)  # OSX eigen3
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN_INCLUDE_DIRS})
add_library(GPMDS src/MultiGPR.cpp src/GPMDS.cpp)
add_executable(expData src/expData.cpp)
target_link_libraries(expData GPMDS)

option(USE_DOUBLE_PRECISION "Use double precision" off)
if(USE_DOUBLE_PRECISION)
add_definitions(-DUSE_DOUBLE_PRECISION)
endif()
