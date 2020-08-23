cmake_minimum_required(VERSION 2.8.3)
project(uvdar_leader_follower)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nav_msgs
  std_msgs
  mrs_lib
  dynamic_reconfigure
)

###########
## Eigen ##
###########
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

#########################
## dynamic reconfigure ##
#########################
generate_dynamic_reconfigure_options(
  cfg/Follower.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp roslib std_msgs nav_msgs mrs_lib 
  LIBRARIES follower VelocityEstimator
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(follower src/follower.cpp)
target_link_libraries(follower
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_library(VelocityEstimator src/velocity_estimator.cpp)
target_link_libraries(VelocityEstimator
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
