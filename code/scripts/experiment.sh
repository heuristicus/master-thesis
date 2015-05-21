#!/bin/bash

features="shot shotcolor pfh fpfh pfhrgb"
fselect="iss susan sift"

process_dirs="/home/michal/Downloads/pcddata/processed/paramtest/iter100"

# /home/michal/Downloads/pcddata/processed/paramtest/ds0_015 /home/michal/Downloads/pcddata/processed/paramtest/dt0,02_mp8000_pp0,001

# needs attention
# /home/michal/Downloads/pcddata/processed/paramtest/dt0_02 

# completed 
# /home/michal/Downloads/pcddata/processed/paramtest/ds0_02 /home/michal/Downloads/pcddata/processed/paramtest/iter500 /home/michal/Downloads/pcddata/processed/paramtest/ds0_02mp_4500 /home/michal/Downloads/pcddata/processed/paramtest/iter50 /home/michal/Downloads/pcddata/processed/paramtest/nr0_04

for dir in $process_dirs; do
    echo $dir
    for selection in $fselect; do
	echo $selection
	for ftype in $features; do
	    echo $ftype
	    echo "running roslaunch feature_extraction feature_extraction.launch cloud:=$dir feature:=$ftype feature_selection:=$selection"
	    roslaunch feature_extraction feature_extraction.launch cloud:=$dir feature:=$ftype feature_selection:=$selection
	done
    done
done
