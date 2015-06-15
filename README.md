# master-thesis
KTH Master's thesis in Systems, Control and Robotics
# Install
The code for this project uses ROS Indigo.

You may need to install some additional packages to get things working.

Documentation is generated by [Doxygen](https://www.doxygen.org). The test framework
uses the [Google C++ Testing Framework](https://code.google.com/p/googletest/).
You can install both of these with

    sudo apt-get install doxygen libgtest-dev

As of writing (03/15) The `libgtest-dev` package on Ubuntu 14.04 does not
install libraries, only the headers. To create the library files you need to
compile them manually (code from
[here](http://www.thebigblob.com/getting-started-with-google-test-on-ubuntu/)
and
[here](http://stackoverflow.com/questions/13513905/how-to-properly-setup-googletest-on-linux)
for some pointers).

    sudo apt-get install cmake # install cmake
    cd /usr/src/gtest
    sudo cmake CMakeLists.txt
    sudo make
    
    # copy or symlink libgtest.a and libgtest_main.a to your /usr/lib folder
    sudo cp *.a /usr/lib

# Compilation

Some of the packages used by the STRANDS project require `qt_build` and
`mongodb_store`. On Ubuntu systems set up according to the
[tutorial](http://wiki.ros.org/indigo/Installation/Ubuntu), this can be done
with

    sudo apt-get install ros-indigo-qt-build
    sudo apt-get install ros-indigo-mongodb-store

You will also require the OpenCV nonfree library. There doesn't seem to be a
package for this in Ubuntu (although `libopencv-nonfree-dev` existed at some
point). To install, download OpenCV and run the following in the top level of
the extracted zip:

    cmake CMakeLists.txt
    make

This will take a while to compile everything. It may be possible to just compile
the nonfree library, but I don't know how. Find where other libraries from
OpenCV sit on your system with `locate --regexp 'libopencv.*.so'`. For me, they
sit in `/usr/lib/x86_64-linux-gnu/`. Copy the library files to that location.

    sudo cp lib/libopencv_nonfree* /usr/lib/x86-64-linux-gnu

And things should compile when `catkin_make` is run in the workspace which
contains the project. To set up a workspace, see the
[ROS tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

# Testing

In the `code/test` directory there are some files which you can use to check if
things are properly configured. To compile use

    catkin_make run_tests

This will also run the tests and give information about which ones passed or
failed (all should pass).

# Data

The data used comes from an internal dataset at KTH which consists of multiple
frames of rooms taken from a single position within the room. The frames are
registered to form a single large cloud of the whole room. Both the full cloud
and individual frames are available. Some information about transforms relating
the frames to the full cloud are found in an XML file.

All clouds start off in their own frame of reference, so the origin is at the
position at which the camera was when the frames were taken. The complete cloud
can be transformed into the map frame using the transform of the first
intermediate cloud from `room.xml`. The rooms are in a corridor, and there are
several rooms for which there are scans available, so doing this puts the cloud
in the correct position in the corridor. To transform the individual frames (e.g.
`intermediate_cloud0001.pcd`) into the map frame, it is necessary to first apply
the transform from the original position to the registered position, and then
apply the same transform as was applied to the complete cloud.

In addition, some annotated clouds of objects present in the rooms are also
available. Each cloud (e.g. `rgb_0013_label_0.pcd`) corresponds to an object
extracted from one of the intermediate clouds, in this case the 13th. Each cloud
has an associated label found in a text file with a single line (e.g.
`rgb_0013_label_0.txt`).

# Usage

## Setup
Some global parameters can be configured in the
[params](code/obj_search/objsearch_toplevel/config/global_params.yaml) file. In
particular, the `raw_data_dir` and `processed_data_dir` should be set to the
location of the raw data, and where the processed data should be output to.
These values are used in certain parts of the system to facilitate the output of
files to the same subdirectories in the processed directory as the raw
directory. So, for example, if a file is in `raw/data/data1/data2`, then we
output corresponding processed data to `processed/data/data1/data2`.

## Preprocessing

The preprocessing node will reduce the size of the cloud by trimming the floors
and ceilings, and removing large planes. It is intended to be run on raw data
from scans of a room from a single position. There are numerous parameters which
can be specified in the
[launch file](code/obj_search/preprocess/launch/preprocess.launch). The default
is to carry out all preprocessing steps: downsampling, plane extraction,
trimming, annotation rotation and normal extraction.

The result of the processing is output to a `processed` directory. If given data
in `/pcddata/raw/annotated/rares/20140901/patrol_run_31/room_2/`, for example,
the resulting processed data will be placed in
`/pcddata/processed/annotated/rares/20140901/patrol_run_31/room_2/`.

### Complete clouds

This command will do preprocessing steps on the complete cloud specified, and
also process the associated annotations.

    roslaunch preprocess preprocess.launch cloud:=/home/michal/Downloads/pcddata/raw/annotated/rares/20140901/patrol_run_31/room_2/complete_cloud.pcd

This command will do preprocessing steps for all complete clouds in
subdirectories of the specified directory.
	
    roslaunch preprocess preprocess.launch cloud:=/home/michal/Downloads/pcddata/raw/annotated/rares/

This command will do preprocessing steps for all clouds in subdirectories of the
specified directory that contain the string "intermediate". You can also
provide regex input in the same position and it should work (untested).
	
    roslaunch preprocess preprocess.launch cloud:=/home/michal/Downloads/pcddata/raw/annotated/rares/ match:=intermediate


### Intermediate clouds
The intended use of the preprocessing on intermediate clouds is to generate
clouds which can be used to check the efficacy of the system. We extract
features from the complete cloud and an intermediate cloud, and then check the
correspondences between the extracted features.

    roslaunch preprocess preprocess.launch
    cloud:=/home/michal/Downloads/pcddata/raw/annotated/rares/20140901/patrol_run_31/room_2/intermediate_cloud0014.pcd

### Annotations
The following will process only annotations, and output them to the specified directory.

    roslaunch preprocess preprocess.launch cloud:=/media/michal/Pauli/masterdata/raw/annotated/rares/ output:=/home/michal/Downloads/pcddata/processed/testing/dsannot/0,01 downsample_leafsize:=0.01 downsample:=false planes:=false trim:=false normals:=false annotations:=true

## Feature Extraction
This will compute shot features using uniform interest point selection on all
files that match `nonPlanes.pcd` in the given cloud directory. The data will be
output to a directory according to the setting of the global parameter
`obj_search/processed_data_dir`. If this is set to
`/home/michal/Downloads/pcddata/processed/`, then an example output feature
cloud would be
`/home/michal/Downloads/pcddata/processed/paramtest/dt0_02/annotated/rares/20140909/patrol_run_59/room_3/features/nonPlanes<shotcolor_uniform_2015-05-26_11-12-20.pcd`,
where the input cloud was
`/home/michal/Downloads/pcddata/processed/paramtest/dt0_02/annotated/rares/20140909/patrol_run_59/room_3/nonPlanes.pcd`.

    roslaunch feature_extraction feature_extraction.launch cloud:=/home/michal/Downloads/pcddata/processed/paramtest/dt0_02 feature:=shot feature_selection:=uniform

One can also specify an output directory using the `output` parameter. With the following command, the cloud above would be output to `/home/michal/dt0_02/annotated/rares/20140909/patrol_run_59/room_3/nonPlanes.pcd`.

    roslaunch feature_extraction feature_extraction.launch cloud:=/home/michal/Downloads/pcddata/processed/paramtest/dt0_02 feature:=shot feature_selection:=uniform output:=/home/michal

If the input path does not match either the global raw data directory
(`obj_search/raw_data_dir`) or the processed directory, then the subpath of that
directory is not extracted, and data will be output directly - if one is
processing multiple clouds then this is not ideal because all the results will
be placed in a single location instead of being separated by directories.


## Object Query

Comparing SHOT features extracted from the 14th intermediate cloud to the
features extracted from the complete cloud.
	
    roslaunch object_query object_query.launch query:=/home/michal/Downloads/pcddata/processed/annotated/rares/20140901/patrol_run_31/room_2/features/0014_nonPlanes_shot.pcd target:=/home/michal/Downloads/pcddata/processed/annotated/rares/20140901/patrol_run_31/room_2/features/nonPlanes_shot.pcd


Compare shot uniform features from the chair 1 object to the same features for
the room cloud, with some modified parameters. Results will be output to the
directory `/home/michal/Downloads/pcddata/processed/query/iter500`. A
subdirectory for `chair1` will be created there.

    roslaunch object_query object_query.launch query:=/home/michal/Downloads/pcddata/processed/testing/querytest/features/rgb_0015_label_chair1\<shot_uniform_2015-05-19_13-24-47.pcd target:=/media/michal/Pauli/masterdata/processed/paramtest/iter500/annotated/rares/20140901/patrol_run_33/room_1/features/nonPlanes\<shot_uniform_2015-05-19_00-51-50.pcd n_max_points:=200 cluster_tolerance:=0.2 K:=5 results_out:=/home/michal/Downloads/pcddata/processed/query/iter500

Run on a target directory, on all feature clouds which match the string `nonPlanes<pfhrgb_sift`.

    roslaunch object_query object_query.launch query:=/home/michal/Downloads/pcddata/processed/query/queryobjects/0,01/top_couch_jacket2/features/rgb_0003_label_top_couch_jacket2<pfhrgb_sift_2015-05-26_15-12-21.pcd target:=/media/michal/Pauli/masterdata/processed/annotated/rares n_max_points:=200 match:=nonPlanes<pfhrgb_sift cluster_tolerance:=0.2 K:=5 results_out:=/home/michal/Downloads/pcddata/processed/query/rares

# Data Processing

## Feature Data

Having computed various types of features using different interest point selection methods, one can use featureprocess.py to aggregate the data into a usable form. The following command creates files in `.` which contain the data from all the files in `.` which have the extension `.txt`, grouped by which feature was used. 

    python scripts/features/featureprocess.py -af . `find . -regex .*.txt | xargs echo`

This command is similar to the above, but instead groups by interest point selection method

    python scripts/features/featureprocess.py -ai . `find . -regex .*.txt | xargs echo`

Finally, this command can be used to create files which contain only information about the timings for interest point selection.

    python scripts/features/featureprocess.py -al . `find . -regex .*.txt | xargs echo`

## Query Data
For a single directory containing query results, with directories for different
object types, can use the following to output results to the current directory.
Data is aggregated according to the feature type used.

    python3 scripts/query/process_results.py -m . `find . -regex .*[0-9].txt`

The switch `-mi` will aggregate according to the interest point type used.
`-m[i]k` will also produce results, but will skip SHOT, PFH, FPFH, SIFT and
SUSAN data.
