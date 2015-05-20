#ifndef ANNOTATION_EXTRACT_H
#define ANNOTATION_EXTRACT_H

#include <algorithm>
#include <map>
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
	 * @param filePath Path to the directory containing processed clouds and
	 * their labels.
	 * @param label this defines an additional filter in the regex used -
	 * can be used to find only files which have this label
	 * @return
	 */
	template <typename PointT>
	std::vector<AnnotatedCloud<PointT> > getProcessedAnnotatedClouds(std::string filePath, std::string label="NULL") {
	    // get two sorted vectors containing the cloud files and txt data for the
	    // annotations. They should be the same length.
	    ROS_INFO("Searching %s for annotation clouds", filePath.c_str());

	    
	    std::string match(".*label" + (label.compare("NULL") == 0 ? ".*.pcd" : "_" + label + ".pcd"));
	    ROS_INFO("Using match string %s", match.c_str());
	    std::vector<std::string> matchesPCD = sysutil::listFilesWithString(
		filePath, std::regex(match));
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
		throw sysutil::objsearchexception("Failed to find file containing annotation label");
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
	 * @return A vector of annotated point clouds with labels extracted from the files
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

	    // if the number of files is not the same, we need to process only
	    // those files which can be paired up
	    if (matchesPCD.size() != matchesTXT.size()) {
		ROS_INFO("Could not find label for one of the annotation clouds");
		throw sysutil::objsearchexception(".txt and .pcd file mismatch for annotations");
		// std::cout << "before remap" << std::endl;
		// for (size_t i = 0; i < std::max(matchesTXT.size(), matchesPCD.size()); i++) {
		//     if (matchesTXT.size() -1 < i) {
		// 	std::cout << "*----*   - " << std::endl;
		//     } else {
		// 	std::cout << matchesTXT[i] << " - ";
		//     }

		//     if (matchesPCD.size() -1 < i) {
		// 	std::cout << "*----*" << std::endl;
		//     } else {
		// 	std::cout << matchesPCD[i] << std::endl;
		//     }
		// }
		
		// std::map<std::string, int> duplimap;
		// // insert elements from both the vectors into the map, stripping
		// // off the extension so that the strings will match.
		// for (size_t i = 0; i < matchesPCD.size(); i++) {
		//     // each of these elements will be unique, so don't need to
		//     // check the return value of insert. Retain the path when
		//     // removing the extension
		//     duplimap.insert(std::pair<std::string,int>(
		// 			sysutil::removeExtension(matchesPCD[i], false), 0));
		    
		// }

		// for (size_t i = 0; i < matchesTXT.size(); i++) {
		//     // attempt to insert the key into the map
		//     std::pair<std::map<std::string, int>::iterator,bool> ret;
		//     ret = duplimap.insert(std::pair<std::string,int>(
		// 			      sysutil::removeExtension(matchesTXT[i], false), 0));
		//     // if the key already existed, then increment the int it
		//     // holds to indicate that this is a paired element
		//     if (ret.second == false) {
		// 	ret.first->second++;
		//     }
		// }

		// // now we should have a map where unpaired elements have a value
		// // of 0. Iterate through it and repopulate the vectors with
		// // elements which are paired.
		// matchesTXT.clear();
		// matchesPCD.clear();
		// for (auto it = duplimap.begin(); it != duplimap.end(); it++) {
		//     if (it->second == 1) {
		// 	// if paired, put the strings into the vectors, putting
		// 	// the extensions back on.
		// 	matchesTXT.push_back(it->first + ".txt");
		// 	matchesPCD.push_back(it->first + ".pcd");
		//     }
		// }

		// std::cout << "after remap" << std::endl;
		// for (size_t i = 0; i < matchesTXT.size(); i++) {
		//     std::cout << matchesTXT[i] << " - " << matchesPCD[i] << std::endl;
		// }


	    }
	    
	    typename std::vector<AnnotatedCloud<PointT> > clouds;

	    pcl::PCDReader reader;
	    // loop through the vectors, creating annotation clouds where the
	    // two files match up. In most cases this should be simple, with
	    // both i and j increasing at the same rate, but if there is a
	    // mismatch, one of the indices will skip ahead to ignore the file
	    for (size_t i = 0; i < matchesPCD.size(); i++) {
		clouds.push_back(getRawAnnotatedCloud<PointT>(matchesPCD[i], matchesTXT[i],
							      reader));
	    }
    
	    return clouds;
	}

    } // namespace pclutil
} // namespace objsearch

#endif // ANNOTATION_EXTRACT_H
