#!/bin/bash

for dirn in `find $1 -maxdepth 1`; do
    echo $dirn
    mv $dirn/features/*.pcd $dirn/points.pcd
    rm -r $dirn/features
done


