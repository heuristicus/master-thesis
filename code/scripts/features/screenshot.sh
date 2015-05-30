#!/bin/bash
indir=$1

for dir in `ls $indir`; do
    echo $dir
    for subd in `ls $dir`; do
	pcl_viewer -bc 1,1,1 -cam $DATA_DIR/camera_params/interest.cam -fc 0,0,0 -ps 2 $indir/$dir/$subd/points.pcd
	mv $indir/screenshot*.png $indir/$dir\_$subd.png
	rm $indir/*.cam
    done
done
