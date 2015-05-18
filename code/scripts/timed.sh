#!/bin/bash

difference=$(($(date -d $1 +%s) - $(date +%s)))
echo "Difference is $difference"
if [ $difference -lt 0 ]
then
    echo "sleeping for $((86400 + difference))"
    sleep $((86400 + difference))
else
    sleep $difference
fi

$2
