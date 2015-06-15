#!/bin/bash

# read the file containing the collected filenames of annotations that we want
# to use, and display them.

while read fname; do
    echo $fname.pcd;
    pcl_viewer $fname.pcd -bc 1,1,1
done < $1;
