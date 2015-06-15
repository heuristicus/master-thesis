#!/bin/bash
# show the top n ($2) clusters from file ($1) in pcl viewer

if [ "$#" -lt 3 ]; then
    echo "Give cluster file, number of clusters to show and what to rename screenshots to"
    exit
fi

echo -e "`sort -nr $1`" > tmp.txt
n=$2
i=0
while read line; do
    if [ "$i" -eq "$n" ]; then
	break
    fi
    
    l=($line)
    pcl_viewer -bc 1,1,1 ${l[3]}
    mv screenshot*.png "$img/queryresults/$3_$i.png"
    rm screenshot*.cam
    ((i++))
done < tmp.txt
