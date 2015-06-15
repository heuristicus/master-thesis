#!/bin/bash

querybase="/home/michal/Downloads/pcddata/processed/query/queryobjects/0,02"
#base="/home/michal/Downloads/pcddata/processed/paramtest"
base="/media/michal/Pauli/masterdata/processed/paramtest"
outbase="/home/michal/Downloads/pcddata/processed/query/k10_mx400_t0,25"
dirs="ds0_02mp_4500" #iter100 nr0_04 ds0,015_mp9000 dt0,02_mp8000_pp0,001 ds0_02mp_4500
features="shot shotcolor pfh pfhrgb fpfh " #shot shotcolor pfh fpfh pfhrgb
fselect="uniform iss susan" #uniform iss susan sift
objects="top_couch_jacket2 backpack2 hanger_jacket laptop1 pillow chair1" #trash_bin top_couch_jacket2 backpack2 hanger_jacket laptop1 pillow chair1
nmax=400
k=10
tol=0.25

var=0
for dir in $dirs; do
    echo $dir
    dt=`echo $dir | sed 's/,//g'`
    for obj in $objects; do
	for selection in $fselect; do
    	    for ftype in $features; do
		qfile=`find $querybase/$obj/features/ -regex .*\<$ftype\_$selection.*`
		matchstr="nonPlanes<${ftype}_$selection"
	     	echo "roslaunch object_query object_query.launch nodename:=$dt`date +%s` query:=$qfile target:=$base/$dir n_max_points:=200 match:=$matchstr cluster_tolerance:=0.2 K:=5 results_out:=$outbase/$dir"
		roslaunch object_query object_query.launch nodename:=$dt`date +%s` query:=$qfile target:=$base/$dir n_max_points:=$nmax match:=$matchstr cluster_tolerance:=$tol K:=$k results_out:=$outbase/$dir
		((var++))
	    done
    	done
    done
done
echo $var
