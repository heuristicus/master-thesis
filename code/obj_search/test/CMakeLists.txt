# from http://www.thebigblob.com/getting-started-with-google-test-on-ubuntu/
# http://docs.ros.org/indigo/api/catkin/html/howto/format2/gtest_configuration.html
cmake_minimum_required(VERSION 2.6)
project(testing)

find_package(catkin REQUIRED COMPONENTS rosunit sysutil)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS rosunit sysutil
#  DEPENDS system_lib
)

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
 
# Link runTests with what we want to test and the GTest and pthread library
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(sysutil_test src/sysutil_test.cpp)
  target_link_libraries(sysutil_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread gtest_main sysutil)
endif()