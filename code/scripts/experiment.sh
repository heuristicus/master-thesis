#!/bin/bash

features="shot shotcolor fpfh pfh pfhrgb"
fselect="uniform iss susan sift"

#process_dirs="/home/michal/Downloads/pcddata/processed/paramtest/dt0_02"
outdir="testing/dsannot"
process_dirs="/home/michal/Downloads/pcddata/processed/testing/dsannot/0,015 /home/michal/Downloads/pcddata/processed/testing/dsannot/0,02"



for dir in $process_dirs; do
    echo $dir
    for selection in $fselect; do
    	for ftype in $features; do
	#    roslaunch feature_extraction feature_extraction.launch cloud:=$dir output:=/home/michal/Downloads/pcddata/processed/$outdir/`basename $dir` feature:=$ftype feature_selection:=$selection match:=label
	    roslaunch feature_extraction feature_extraction.launch cloud:=$dir feature:=$ftype feature_selection:=$selection match:=label
    	done
    done
done
