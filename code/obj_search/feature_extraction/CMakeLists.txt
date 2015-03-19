cmake_minimum_required(VERSION 2.8.3)
project(feature_extraction)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++1y -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  rosutil
  sysutil
)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs pcl_ros rosutil sysutil
#  DEPENDS system_lib
)

include_directories(include ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(extract_features src/featureExtraction.cpp)
target_link_libraries(extract_features ${catkin_LIBRARIES} ${PCL_LIBRARIES} rosutil sysutil)
add_dependencies(extract_features ${catkin_EXPORTED_TARGETS})