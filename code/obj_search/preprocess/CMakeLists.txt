cmake_minimum_required(VERSION 2.8.3)
project(preprocess)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  rosutil
)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs pcl_ros rosutil
#  DEPENDS system_lib
)

include_directories(include ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(planetest src/planeTest.cpp)
target_link_libraries(planetest ${catkin_LIBRARIES} ${PCL_LIBRARIES} rosutil)
add_dependencies(planetest ${catkin_EXPORTED_TARGETS})