#ifndef CLOUD_VIEWER_H
#define CLOUD_VIEWER_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

namespace objsearch {
    namespace pclutil {

	pcl::visualization::PCLVisualizer::Ptr createVisualiser() {
	    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	    viewer->setBackgroundColor(0, 0, 0);
	    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	    viewer->addCoordinateSystem(1.0);
	    viewer->initCameraParameters();

	    return viewer;
	}

	void addPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
			   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
	    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
	}

	void addPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
			   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	    viewer->addPointCloud<pcl::PointXYZ>(cloud);
	}

	void waitForExit(pcl::visualization::PCLVisualizer::Ptr& viewer){
	}
	
    } // namespace pclutil
} // namespace objsearch

#endif // CLOUD_VIEWER_H
