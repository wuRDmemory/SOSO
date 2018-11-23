cmake_minimum_required(VERSION 2.8.3)
project(robotic_optimizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotic_optimizer
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

find_package(OpenCV REQUIRED)
if(${OpenCV_FIND})
    message(">>> We find OpenCV, version is ${OpenCV_VERSION}")
endif()

find_package(Ceres REQUIRED)
if (Ceres_FOUND)
    message(">> we find ceres, version is ${Ceres_VERSION}, aaa")
    include_directories(${CERES_INCLUDE_DIRS})
endif ()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  /usr/inlcude/eigen3
)

SET(SOURCE src/robot.cpp src/optimizer.cpp src/visualization.cpp)

add_executable(test_node src/robotic_optimizer_node.cpp ${SOURCE})
target_link_libraries(test_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
