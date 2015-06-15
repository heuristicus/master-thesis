#!/bin/bash

indir=$1

for dir in `ls $indir`; do
    check=$indir/$dir/query_results
    python3 $scr/query/process_results.py -m $check `find $check -regex .*query.*[0-9].txt`
    python3 $scr/query/process_results.py -mk $check `find $check -regex .*query.*[0-9].txt`
    python3 $scr/query/process_results.py -mi $check `find $check -regex .*query.*[0-9].txt`
    python3 $scr/query/process_results.py -mik $check `find $check -regex .*query.*[0-9].txt`
    
done
