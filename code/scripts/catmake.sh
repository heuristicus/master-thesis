#!/bin/bash
pushd ~/catkin_ws
# call catkin make in the workspace with the arguments given to the script
catkin_make $@ --cmake-args -DCMAKE_CXX_FLAGS="-Wall"
popd
