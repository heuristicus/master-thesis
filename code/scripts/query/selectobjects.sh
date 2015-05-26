#!/bin/bash

objectloc="/media/michal/Pauli/masterdata/processed/annotated/rares/"
# these objects will be removed from consideration - need to be specific with these (especially couch)
#removeobjects_regex="backpack[134] person bag chair[2345] label_couch_jacket* hanger_jacket[234] top_couch_jacket[$|34] laptop2 pillow[23] can couch($|.pcd)"
keepobjects_regex="chair1 backpack2 trash_bin laptop1 hanger_jacket($|.pcd) pillow($|.pcd) top_couch_jacket2 helmet mug1"

# allannotations has filenames without the full path
for file in `find $objectloc ! -regex .*normals.pcd | grep label | xargs echo`; do
    fname=$(basename "$file");
    fname="${fname%.*}"
    echo $fname
done > allannotations.txt

# fullannotations has filenames with full path
find /media/michal/Pauli/masterdata/processed/annotated/rares/ ! -regex .*normals.pcd | grep label > fullannotations.txt

# all the unique labels in this file
sed -r 's/^.{15}//' allannotations.txt | sort | uniq > uniqueannotations.txt

rm occurrences.txt
for object in `cat uniqueannotations.txt`; do
    echo $object `grep -E label_$object\$ allannotations.txt | wc -l` >> occurrences.txt
done

sort -nrk2 occurrences.txt > occurrences_sorted.txt

rm allannotations_trim.txt
rm fullannotations_trim.txt
touch allannotations_trim.txt
touch fullannotations_trim.txt

for item in $keepobjects_regex; do
    grep -E $item allannotations.txt >> allannotations_trim.txt
    grep -E $item fullannotations.txt >> fullannotations_trim.txt
done

# # copy data into temporary files so it doesn't get destroyed, even though we
# # don't really care
# cat fullannotations.txt > full_tmp2.txt
# cat allannotations.txt > all_tmp2.txt

# allcur=all_tmp1.txt
# allprev=all_tmp2.txt
# fullcur=full_tmp1.txt
# fullprev=full_tmp2.txt

# for item in $removeobjects_regex; do
#     grep -Ev $item $allprev >> $allcur
#     grep -Ev $item $fullprev >> $fullcur

#     # swap files so that one is read into the other
#     tmp=$fullcur
#     fullcur=$fullprev
#     fullprev=$tmp
#     tmp=$allcur
#     allcur=$allprev
#     allprev=$tmp
# done

# cat $allprev > allannotations_trim.txt
# cat $fullprev > fullannotations_trim.txt

# rm $allcur $fullcur $allprev $fullprev

sed -r 's/^.{15}//' allannotations_trim.txt | sort | uniq > uniqueannotations_trim.txt

uqtmp=`cat uniqueannotations_trim.txt`

for object in $uqtmp; do
    pcdfiles=`grep $object.pcd fullannotations_trim.txt`
    echo "" > "selected_$object.txt"
    for file in $pcdfiles; do
    	pcl_viewer $file
	read -p "Keep this file? " -n 1 -r
	echo    # (optional) move to a new line
	if [[ ! $REPLY =~ ^[Yys]$ ]]
	then
    	    continue
	fi
	if [[ "$REPLY" = "s" ]]; then # s to skip this object altogether
	    break
	fi
	echo $file >> "selected_$object.txt"
    done

done

# take the items from a directory and put them into a place here
while read itm; do
    bn=`basename $itm`
    bn="${bn%.*}"
    pr=`dirname $itm`
    label=`echo $bn | sed -r 's/^.{15}//'`
    short=`echo $pr | sed -r 's/^.{57}//'`
    reg=".*$short.*$bn.*"
    fn=`find ~/Downloads/pcddata/processed/testing/dsannot/0\,01/annotated/rares/ -regex $reg`
    mkdir ../0,01/$label
    cp $fn ../0,01/$label
    mkdir ../0,01/$label/features
    mv "../0,01/$label/"*"<"* "../0,01/$label/features/"
done < collected.txt
