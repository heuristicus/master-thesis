cmake_minimum_required(VERSION 2.8.3)
project(preprocess)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++1y -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  rosutil
  sysutil
  metaroom_xml_parser
)

rosbuild_prepare_qt4(QtCore QtXml)

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

add_executable(preprocess src/preprocess.cpp)
target_link_libraries(preprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES} rosutil sysutil metaroom_xml_parser)
add_dependencies(preprocess ${catkin_EXPORTED_TARGETS})