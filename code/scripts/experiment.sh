#!/bin/bash

features="shot shotcolor pfh fpfh pfhrgb"
fselect="uniform iss susan sift"

#process_dirs="/home/michal/Downloads/pcddata/processed/paramtest/dt0_02"
outdir=""
process_dirs="/media/michal/Pauli/masterdata/processed/annotated/"

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RESET RAW DATA DIR IN GLOBAL PARAMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

for dir in $process_dirs; do
    echo $dir
    for selection in $fselect; do
    	for ftype in $features; do
	#    roslaunch feature_extraction feature_extraction.launch cloud:=$dir output:=/home/michal/Downloads/pcddata/processed/$outdir/`basename $dir` feature:=$ftype feature_selection:=$selection match:=label
	    echo "roslaunch feature_extraction feature_extraction.launch cloud:=$dir feature:=$ftype feature_selection:=$selection"
	    roslaunch feature_extraction feature_extraction.launch cloud:=$dir output:=/home/michal/Downloads/pcddata/processed/deffeature/$dir feature:=$ftype feature_selection:=$selection
    	done
    done
done
