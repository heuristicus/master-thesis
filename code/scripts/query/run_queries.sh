#!/bin/bash

querybase="/home/michal/Downloads/pcddata/processed/query/queryobjects/0,01"
base="/media/michal/Pauli/masterdata/processed/paramtest/"
outbase="/home/michal/Downloads/pcddata/processed/query/k5_mx200_t0,2"
dirs="iter100" #iter100 nr0_04 ds0,015_mp9000 dt0,02_mp8000_pp0,001 ds0_02mp_4500
features="shot shotcolor pfh fpfh pfhrgb" #shot shotcolor pfh fpfh pfhrgb
fselect="iss susan uniform"
objects="trash_bin" #trash_bin top_couch_jacket2 backpack2 hanger_jacket laptop1 pillow
nmax=200
k=5
tol=0.2

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
