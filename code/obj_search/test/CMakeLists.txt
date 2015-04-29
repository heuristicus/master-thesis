# from http://www.thebigblob.com/getting-started-with-google-test-on-ubuntu/
# http://docs.ros.org/indigo/api/catkin/html/howto/format2/gtest_configuration.html
cmake_minimum_required(VERSION 2.6)
project(testing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++1y -Wall")

find_package(catkin REQUIRED COMPONENTS rosunit sysutil pclutil pcl_ros)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS rosunit sysutil pclutil pcl_ros
#  DEPENDS system_lib
)

# Locate GTest
find_package(GTest REQUIRED)
find_package(PCL REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# without this the pclutil test will not compile correctly (undefined references)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
# Link runTests with what we want to test and the GTest and pthread library
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(sysutil_test src/sysutil_test.cpp)
  target_link_libraries(sysutil_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread gtest_main sysutil)
  
  catkin_add_gtest(pclutil_test src/pclutil_test.cpp)
  target_link_libraries(pclutil_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${PCL_LIBRARIES} pthread gtest_main pclutil)
endif()