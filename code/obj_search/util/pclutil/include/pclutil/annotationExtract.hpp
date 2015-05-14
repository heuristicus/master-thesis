#ifndef ANNOTATION_EXTRACT_H
#define ANNOTATION_EXTRACT_H

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include <exception>

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
	AnnotatedCloud<PointT> getProcessedAnnotatedCloud(
	    std::string filename, pcl::PCDReader& reader=pcl::PCDReader()) {
	    // the label name comes between "label_" and the extension in
	    // the filename
	    std::string label = sysutil::removeExtension(filename, true);
	    int labelStartInd = label.find("label_") + 6;
	    // extract the label from the filename
	    label = std::string(label.begin() + labelStartInd, label.end());
		
	    // ROS_INFO("----------%s----------", filename.c_str());
	    // ROS_INFO("File label is %s", label.c_str());

	    // create a new cloud each time to get a different pointer.
	    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	    reader.read(filename, *cloud);
//	    ROS_INFO("Cloud size is %d", (int)cloud->size());
	    return AnnotatedCloud<PointT>(label, filename, cloud);
	}

	/** 
	 * Get a vector of point clouds with their corresponding labels from the
	 * given directory. Each cloud and label should have "label" somewhere in
	 * the filename, followed by the label for points in that cloud, e.g.
	 * rgb_0014_label_chair1.pcd. These files are automatically created by
	 * preprocessing.
	 * 
	 * @param filePath Path to the directory containing processed clouds and their labels.
	 * 
	 * @return 
	 */
	template <typename PointT>
	std::vector<AnnotatedCloud<PointT> > getProcessedAnnotatedClouds(std::string filePath) {
	    // get two sorted vectors containing the cloud files and txt data for the
	    // annotations. They should be the same length.
	    ROS_INFO("Searching %s for annotation clouds", filePath.c_str());
	    
	    std::vector<std::string> matchesPCD = sysutil::listFilesWithString(
		filePath, std::regex(".*label.*pcd"));
	    std::sort(matchesPCD.begin(), matchesPCD.end());

	    ROS_INFO("%d matches", (int)matchesPCD.size());
	    
	    typename std::vector<AnnotatedCloud<PointT> > clouds;

	    pcl::PCDReader reader;
	    for (size_t i = 0; i < matchesPCD.size(); i++) {
		clouds.push_back(getProcessedAnnotatedCloud<PointT>(matchesPCD[i], reader));
	    }
    
	    return clouds;
	}

	template <typename PointT>
	AnnotatedCloud<PointT> getRawAnnotatedCloud(
	    std::string filename, std::string labelfile,
	    pcl::PCDReader& reader=pcl::PCDReader()) {
	    std::ifstream file;
	    // open the text file and extract the label
	    file.open(labelfile);
	    std::string label;
	    if (file.is_open()){
		std::getline(file, label);
		file.close();
	    } else {
		ROS_INFO("Failed to open file %s", labelfile.c_str());
		throw std::exception();
	    }

	    // ROS_INFO("----------%s----------", filename.c_str());
	    // ROS_INFO("File label is %s", label.c_str());
	    // create a new cloud each time to get a different pointer.
	    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	    reader.read(filename, *cloud);
	    // ROS_INFO("Cloud size is %d", (int)cloud->size());
	    return AnnotatedCloud<PointT>(label, filename, cloud);
    	}
	
	/** 
	 * Get a vector of point clouds with their corresponding labels from the given
	 * directory. Each cloud and label should have "label" somewhere in the
	 * filename, and sorting the files should mean that when paired the correct
	 * label .txt file is paired with its .pcd file.
	 * 
	 * @param filePath Path to the directory containing the raw clouds and their labels.
	 * 
	 * @return 
	 */
	template <typename PointT>
	std::vector<AnnotatedCloud<PointT> > getRawAnnotatedClouds(std::string filePath) {
	    // get two sorted vectors containing the cloud files and txt data for the
	    // annotations. They should be the same length.
	    std::vector<std::string> matchesPCD = sysutil::listFilesWithString(
		filePath, std::regex(".*label.*pcd"));
	    std::sort(matchesPCD.begin(), matchesPCD.end());

	    std::vector<std::string> matchesTXT = sysutil::listFilesWithString(
		filePath, std::regex(".*label.*txt"));
	    std::sort(matchesTXT.begin(), matchesTXT.end());

	    if (matchesPCD.size() != matchesTXT.size()) {
		ROS_INFO("Different numbers of .pcd files and .txt files for annotations. Should be the same.");
		throw std::exception();
	    }

	    typename std::vector<AnnotatedCloud<PointT> > clouds;

	    pcl::PCDReader reader;
	    for (size_t i = 0; i < matchesPCD.size(); i++) {
		clouds.push_back(getRawAnnotatedCloud<PointT>(matchesPCD[i], matchesTXT[i],
							      reader));
	    }
    
	    return clouds;
	}

    } // namespace pclutil
} // namespace objsearch

#endif // ANNOTATION_EXTRACT_H
