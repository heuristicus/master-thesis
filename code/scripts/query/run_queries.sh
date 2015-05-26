#!/bin/bash

querybase="/home/michal/Downloads/pcddata/processed/query/queryobjects/0,01"
base="/media/michal/Pauli/masterdata/processed/paramtest"
outbase="/home/michal/Downloads/pcddata/processed/query"
dirs="dt0,02_mp8000_pp0,001" # iter100 iter50 iter500 nr0_04 #ds0_015 ds0_02mp_4500 
features="shot shotcolor fpfh pfh pfhrgb"
fselect="uniform iss susan sift"
objects="backpack2 chair1 hanger_jacket helmet laptop1 mug1 pillow top_couch_jacket2 trash_bin"

var=0
for dir in $dirs; do
    echo $dir
    for obj in $objects; do
	for selection in $fselect; do
    	    for ftype in $features; do
		qfile=`find $querybase/$obj/features/ -regex .*$ftype\_$selection.*`
		matchstr="nonPlanes<${ftype}_$selection"
		roslaunch object_query object_query.launch query:=$qfile target:=$base/$dir n_max_points:=200 match:=$matchstr cluster_tolerance:=0.2 K:=5 results_out:=$outbase/$dir
		((var++))
	    done
    	done
    done
done
echo $var
