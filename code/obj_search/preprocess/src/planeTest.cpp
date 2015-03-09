#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "rosutil/rosutil.hpp"

#include <iostream>
#include <string>
#include <cstdlib>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "planetest");
    ros::NodeHandle handle;

    std::string cloudFile;
    ROSUtil::getParam(handle, "/planetest/cloud_file", cloudFile);
    float ransacDistanceThresh;
    ROSUtil::getParam(handle, "/planetest/RANSAC_distance_threshold", ransacDistanceThresh);
    int ransacIterations;
    ROSUtil::getParam(handle, "/planetest/RANSAC_iterations", ransacIterations);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloudFile, *cloud) != -1){
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

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    for (size_t i = 0; i < inliers->indices.size(); i++) {
	cloud->points[inliers->indices[i]].r = 255;
	cloud->points[inliers->indices[i]].b = 255;
    }


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("Cloud viewer"));;
    std::string cloudName("cloud");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud(cloud, rgb, cloudName.c_str());
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudName.c_str());
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    while (!viewer->wasStopped ())
    {
	viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
