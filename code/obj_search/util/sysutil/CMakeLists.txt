cmake_minimum_required(VERSION 2.8.3)
project(sysutil)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++1y -Wall")

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

ADD_LIBRARY(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)