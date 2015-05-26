#!/bin/bash

if [ "$#" -lt "2" ]; then
    echo "not enough args"
    exit
fi

datafile=$1
ntoview=$2



nviewed=0
while read line; do
    if [ "$files" = true ] && [ ${line::1} != "#" ]; then
	echo "Viewed $nviewed so far"
	pcl_viewer `echo $line | awk '{print $2,$4}'`
	((nviewed++))
    fi

    if [ "$nviewed" -eq "$ntoview" ]; then
	break
    fi

    if [[ "$line" == "BEGIN_FILES" ]]; then
	files=true
    fi
done < $datafile
