/**
 * @file   planeTest.cpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Mon Mar  9 14:06:36 2015
 * 
 * @brief  Some prototype code for doing preprocessing on point clouds.
 * 
 * 
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "rosutil/rosutil.hpp"
#include "sysutil/sysutil.hpp"
#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <string>
#include <cstdlib>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "planetest");
    ros::NodeHandle handle;

    std::string cloudFile;
    ROSUtil::getParam(handle, "/planetest/cloud_file", cloudFile);
    std::string dataPath;
    ROSUtil::getParam(handle, "/obj_search/raw_data_dir", dataPath);
    // If the given cloud file corresponds to a file in the raw data directory,
    // extract the remaining directories in the path of the file so that the
    // data can be put into the output directory with the same path.
    std::string dataSubDir;
    if (cloudFile.compare(0, dataPath.size(), dataPath) == 0){
	dataSubDir = std::string(cloudFile, dataPath.size());
    }

    std::cout << dataSubDir << std::endl;
    
    exit(0);

    std::string outDir;
    ROSUtil::getParam(handle, "/planetest/output_dir", outDir);
    // If output is not specified, set the output directory to be the processed
    // data directory specified by the global parameters.
    if (std::string("NULL").compare(outDir) == 0) {
	ROSUtil::getParam(handle, "/obj_search/processed_data_dir", outDir);
    }

    float ransacDistanceThresh;
    ROSUtil::getParam(handle, "/planetest/RANSAC_distance_threshold", ransacDistanceThresh);
    int ransacIterations;
    ROSUtil::getParam(handle, "/planetest/RANSAC_iterations", ransacIterations);
    int planesToExtract;
    ROSUtil::getParam(handle, "/planetest/planes_to_extract", planesToExtract);

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloudFile, *originalCloud) != -1){
	std::cout << "Loaded cloud from " << cloudFile.c_str() << std::endl;
    } else {
	std::cout << "Could not load cloud from " << cloudFile.c_str() << std::endl;
	exit(1);
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransacDistanceThresh);
    seg.setMaxIterations(ransacIterations);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // Points will be removed from this cloud - at the end of the process it
    // will contain all the points which were not extracted by the segmentation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intermediateCloud (originalCloud);
    // At each stage, the inliers of the plane model will be extracted to this
    // cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractedPlane (new pcl::PointCloud<pcl::PointXYZRGB>);
    // All of the points that are extracted throughout the process will end up
    // in this cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr allPlanes (new pcl::PointCloud<pcl::PointXYZRGB>);
    // The points which are not inliers to the plane will be placed into this
    // cloud and then swapped into intermediateCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr remainingPoints (new pcl::PointCloud<pcl::PointXYZRGB>);

    ROS_INFO("Starting plane extraction.");
    for (int i = 0; i < planesToExtract; i++) {
	ROS_INFO("Extracting plane %d", i + 1);
	seg.setInputCloud(intermediateCloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size () == 0) {
	    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	    break;
	}


	// Extract the inliers
	extract.setInputCloud(intermediateCloud);
	extract.setIndices(inliers);
	extract.setNegative(false); // Extract the points which are inliers
	extract.filter(*extractedPlane);
	*allPlanes += *extractedPlane; // Add the extracted inliers to the cloud of all planes
	
	// Extract non-inliers
	extract.setNegative(true); // Extract the points which are not inliers
	extract.filter(*remainingPoints);
	intermediateCloud.swap(remainingPoints);
    }
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB>(outDir + "allPlanes.pcd", *allPlanes, false);
    writer.write<pcl::PointXYZRGB>(outDir + "nonPlanes.pcd", *remainingPoints, false);

    // colour the inliers so we can tell them apart easily
    for (auto it = allPlanes->begin(); it != allPlanes->end(); it++) {
	it->r = 255;
	it->b = 255;
    }

    // After the process is finished, combine the extracted planes and the other
    // points together again so that they can be displayed
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *fullCloud = *allPlanes + *remainingPoints;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Cloud viewer"));
    std::string cloudName("cloud");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(fullCloud);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud(fullCloud, rgb, cloudName.c_str());
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudName.c_str());
    viewer->initCameraParameters();
    
    while (!viewer->wasStopped()) {
	viewer->spinOnce(100);
	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
