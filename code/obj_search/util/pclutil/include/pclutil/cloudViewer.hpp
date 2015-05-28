#ifndef CLOUD_VIEWER_H
#define CLOUD_VIEWER_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include "pclutil/colourConversion.hpp"

#include <string>
#include <iostream>
#include <vector>

namespace objsearch {
    namespace pclutil {

	std::string viewerHelp("-n cloud normals : display a cloud with its normals.\n"
			       "-m[clo] cloud ... [camera_params] : display multiple clouds. If c is added, the last cloud is used as the camera params. l will automatically colour clouds - blue, yellow, magenta, orange. Subsequent clouds are random colours. o displays the origin");

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
