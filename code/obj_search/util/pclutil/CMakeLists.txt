cmake_minimum_required(VERSION 2.8.3)
project(pclutil)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++1y -Wall")

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  sysutil
  metaroom_xml_parser
)

find_package(PCL 1.7 REQUIRED)
# to use this, add the location of the directory that pcl 1.7.2 was installed to
# $CMAKE_PREFIX_PATH
#find_package(PCL 1.7.2 REQUIRED)

# needed for metaroom xml
rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
  CATKIN_DEPENDS pcl_ros sysutil
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(include ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(create_cloud src/cloudCreator.cpp)
target_link_libraries(create_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} sysutil)
add_dependencies(create_cloud ${catkin_EXPORTED_TARGETS})

add_executable(view_cloud src/cloudViewer.cpp)
target_link_libraries(view_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(view_cloud ${catkin_EXPORTED_TARGETS})

add_executable(merge_cloud src/cloudMerger.cpp)
target_link_libraries(merge_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} metaroom_xml_parser sysutil)
add_dependencies(merge_cloud ${catkin_EXPORTED_TARGETS})

add_executable(rotate_cloud src/cloudRotator.cpp)
target_link_libraries(rotate_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} metaroom_xml_parser sysutil)
add_dependencies(rotate_cloud ${catkin_EXPORTED_TARGETS})

add_executable(cloud_bounds src/cloudBounds.cpp)
target_link_libraries(cloud_bounds ${catkin_LIBRARIES} ${PCL_LIBRARIES} sysutil)
add_dependencies(cloud_bounds ${catkin_EXPORTED_TARGETS})

add_library(pclutil src/annotationExtract.cpp src/cloudBounds.cpp)
# need to link pcl libraries, otherwise stuff (visualiser, other PCL stuff) breaks in objectQuery
target_link_libraries(pclutil ${catkin_LIBRARIES} ${PCL_LIBRARIES} sysutil)