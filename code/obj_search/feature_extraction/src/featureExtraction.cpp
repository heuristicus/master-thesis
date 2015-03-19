/**
 * @file   featureExtraction.cpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Thu Mar 19 14:12:23 2015
 * 
 * @brief  
 * 
 * 
 */

#include "sysutil/sysutil.hpp"
#include "rosutil/rosutil.hpp"

#include <string>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/usc.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "feature_extraction");
    ros::NodeHandle handle;

    std::string cloudFile;
    // Retrieve the directory containing the cloud to be processed
    ROSUtil::getParam(handle, "/feature_extraction/input_cloud", cloudFile);

    std::string dataPath;
    ROSUtil::getParam(handle, "/obj_search/raw_data_dir", dataPath);
    // If the given cloud file corresponds to a file in the raw data directory,
    // extract the remaining directories in the path of the file so that the
    // data can be put into the output directory with the same path.
    std::string dataSubDir;
    if (cloudFile.compare(0, dataPath.size(), dataPath) == 0){
	dataSubDir = SysUtil::trimPath(std::string(cloudFile, dataPath.size()), 1);
    }

    std::string outDir;
    ROSUtil::getParam(handle, "/feature_extraction/output_dir", outDir);
    // If output is not specified, set the output directory to be the processed
    // data directory specified by the global parameters.
    if (std::string("NULL").compare(outDir) == 0) {
	ROSUtil::getParam(handle, "/obj_search/processed_data_dir", outDir);
    }

    // The output path for processed clouds is the subdirectory combined
    // with the top level output directory. If dataSubDir is not
    // initialised, then clouds are simply output to the top level
    // output directory
    std::string outPath = SysUtil::combinePaths(outDir, dataSubDir);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloudFile, *cloud) != -1){
	std::cout << "Loaded cloud from " << cloudFile.c_str() << std::endl;
    } else {
	std::cout << "Could not load cloud from " << cloudFile.c_str() << std::endl;
	exit(1);
    }

    // The shape context uses xyz points, so need to convert the cloud into that format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    
    cloud_xyz->points.resize(cloud->size());
    int added = 0;

    ROS_INFO("Cloud size: %d", (int)cloud->size());
    // Some of the points in the cloud have nan or inf values, need to strip
    // those to avoid errors. This is only true for the intermediate clouds
    for (size_t i = 0; i < cloud->points.size(); i++) {
	if (std::isnan(cloud->points[i].x) || std::isinf(cloud->points[i].x)
	    || std::isnan(cloud->points[i].y) || std::isinf(cloud->points[i].y)
	    || std::isnan(cloud->points[i].z) || std::isinf(cloud->points[i].z)){
	    continue;
	}
	cloud_xyz->points[i].x = cloud->points[i].x;
	cloud_xyz->points[i].y = cloud->points[i].y;
	cloud_xyz->points[i].z = cloud->points[i].z;
	added++;
    }
    ROS_INFO("Total invalid points: %d", ((int)cloud->size()) - added);
    
    cloud_xyz->points.resize(added);
    
    
    pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors(new pcl::PointCloud<pcl::ShapeContext1980>());
 
    pcl::UniqueShapeContext<pcl::PointXYZ, pcl::ShapeContext1980, pcl::ReferenceFrame> usc;
    usc.setInputCloud(cloud_xyz);
    // Search radius, to look for neighbors. It will also be the radius of the support sphere.
    usc.setRadiusSearch(0.05);
    // The minimal radius value for the search sphere, to avoid being too sensitive
    // in bins close to the center of the sphere.
    usc.setMinimalRadius(0.05 / 10.0);
    // Radius used to compute the local point density for the neighbors
    // (the density is the number of points within that radius).
    usc.setPointDensityRadius(0.05 / 5.0);
    // Set the radius to compute the Local Reference Frame.
    usc.setLocalRadius(0.05);

    ROS_INFO("Computing descriptors.");
    usc.compute(*descriptors);
    ROS_INFO("Done.");
    return 0;
}
