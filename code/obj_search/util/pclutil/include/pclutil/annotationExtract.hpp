#ifndef ANNOTATION_EXTRACT_H
#define ANNOTATION_EXTRACT_H

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

#include <ros/console.h>

#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>

#include "sysutil/sysutil.hpp"

namespace objsearch {
    namespace pclutil {
	template <typename PointT>
	struct AnnotatedCloud {
	    AnnotatedCloud(std::string _label, std::string _fname,
			   typename pcl::PointCloud<PointT>::Ptr _cloud)
		: label(_label), fname(_fname), cloud(_cloud) {}
	    std::string label;
	    std::string fname;
	    typename pcl::PointCloud<PointT>::Ptr cloud;
	};

	template <typename PointT>
	std::vector<AnnotatedCloud<PointT> > getProcessedAnnotatedClouds(std::string filePath);
	template <typename PointT>
	std::vector<AnnotatedCloud<PointT> > getRawAnnotatedClouds(std::string filePath);
    } // namespace pclutil
} // namespace objsearch

#endif // ANNOTATION_EXTRACT_H
