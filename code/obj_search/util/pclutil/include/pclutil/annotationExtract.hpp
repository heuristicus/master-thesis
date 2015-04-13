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

#include <load_utilities.h>

#include "sysutil/sysutil.hpp"

template <typename PointT>
struct AnnotatedCloud {
    AnnotatedCloud(std::string _label, typename pcl::PointCloud<PointT>::Ptr _cloud)
	: label(_label), cloud(_cloud) {}
    std::string label;
    typename pcl::PointCloud<PointT>::Ptr cloud;
};

template <typename PointT>
std::vector<AnnotatedCloud<PointT> > getAnnotatedClouds(std::string filePath);
