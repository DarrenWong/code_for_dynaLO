cmake_minimum_required(VERSION 2.8.3)
project(dynaLO)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
  pcl_conversions
  pcl_ros
  tf_conversions
)
find_package(Gflags)
find_package(Glog)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

find_package(Eigen3)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES}) # Not strictly necessary as Eigen is head only

# Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL Ceres
  INCLUDE_DIRS include
)

set(EXT_LIBS
  ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)

add_executable(odom_estimation_node src/odomEstimationNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/odomEstimationClass.cpp)
target_link_libraries(odom_estimation_node ${EXT_LIBS})

add_executable(laser_mapping_node src/laserMappingNode.cpp src/laserMappingClass.cpp src/lidar.cpp)
target_link_libraries(laser_mapping_node ${EXT_LIBS})

add_executable(laser_processing_node src/processorNode.cpp src/pointProcessor.cpp)
target_link_libraries(laser_processing_node
  ${EXT_LIBS} ${GLOG_LIBRARIES} ${OpenCV_LIBRARIES}
)