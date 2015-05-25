#!/bin/bash

base="/home/michal/Dropbox/study/university/kth/thesis/data/features"
dirs="ds0_02mp_4500 ds0_02 iter500 iter50 nr0_04 iter100 dt0,02_mp8000_pp0,001 ds0_015"
types="uniform sift iss susan"

# needs attention
# default dt0_02

for dir in $dirs; do
    # show directories which contain interest point data and timings
    #python featureprocess.py -h $base/$dir/featuredata*

    # fix files so that they contain all data that is relevant
    python featureprocess.py -f $base/$dir/featuredata*
done

# mkdir "$base/aggregate"
# python featureprocess.py -af $base/aggregate/ `find ../../../data/features/ -regex .*fixed.* | xargs echo`

# python featureprocess.py -ai $base/aggregate/ `find ../../../data/features/ -regex .*fixed.* | xargs echo`
