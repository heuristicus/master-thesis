cmake_minimum_required(VERSION 2.8.3)
project(object_query)

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

add_executable(query_object src/objectQuery.cpp)
target_link_libraries(query_object ${catkin_LIBRARIES} ${PCL_LIBRARIES} rosutil sysutil)
add_dependencies(query_object ${catkin_EXPORTED_TARGETS})