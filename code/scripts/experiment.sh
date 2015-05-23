#!/bin/bash

features="fpfh pfhrgb"
fselect="iss"

process_dirs="/media/michal/Pauli/masterdata/processed/paramtest/iter500 /media/michal/Pauli/masterdata/processed/paramtest/ds0_015"

for dir in $process_dirs; do
    echo $dir
    for selection in $fselect; do
    	for ftype in $features; do
	    roslaunch feature_extraction feature_extraction.launch cloud:=$dir output:=/home/michal/Downloads/pcddata/processed/annfeature/`basename $dir` feature:=$ftype feature_selection:=$selection match:=label
    	done
    done
done
