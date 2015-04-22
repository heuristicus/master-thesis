#include "pclutil/annotationExtract.hpp"

namespace objsearch {
    namespace pclutil {

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
	    
	    std::vector<std::string> matchesPCD = SysUtil::listFilesWithString(
		filePath, std::regex(".*label.*pcd"));
	    std::sort(matchesPCD.begin(), matchesPCD.end());

	    ROS_INFO("%d matches", (int)matchesPCD.size());
	    
	    typename std::vector<AnnotatedCloud<PointT> > clouds;
	    pcl::PCDReader reader;
	    for (size_t i = 0; i < matchesPCD.size(); i++) {
		// the label name comes between "label_" and the extension in
		// the filename
		std::string label = SysUtil::removeExtension(matchesPCD[i], true);
		int labelStartInd = label.find("label_") + 6;
		// extract the label from the filename
		label = std::string(label.begin() + labelStartInd, label.end());
		
		ROS_INFO("----------%s----------", matchesPCD[i].c_str());
		ROS_INFO("File label is %s", label.c_str());

		// create a new cloud each time to get a different pointer.
		typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
		reader.read(matchesPCD[i], *cloud);
		ROS_INFO("Cloud size is %d", (int)cloud->size());
		clouds.push_back(AnnotatedCloud<PointT>(label, matchesPCD[i], cloud));
	    }
    
	    return clouds;
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
	    std::vector<std::string> matchesPCD = SysUtil::listFilesWithString(
		filePath, std::regex(".*label.*pcd"));
	    std::sort(matchesPCD.begin(), matchesPCD.end());

	    std::vector<std::string> matchesTXT = SysUtil::listFilesWithString(
		filePath, std::regex(".*label.*txt"));
	    std::sort(matchesTXT.begin(), matchesTXT.end());

	    if (matchesPCD.size() != matchesTXT.size()) {
		ROS_INFO("Different numbers of .pcd files and .xml files for annotations. Should be the same.");
		exit(1);
	    }

	    typename std::vector<AnnotatedCloud<PointT> > clouds;

	    std::ifstream file;
	    pcl::PCDReader reader;
	    for (size_t i = 0; i < matchesPCD.size(); i++) {
		// open the text file and extract the label
		file.open(matchesTXT[i]);
		std::string label;
		if (file.is_open()){
		    std::getline(file, label);
		    file.close();
		} else {
		    ROS_INFO("Failed to open file %s", matchesTXT[i].c_str());
		    exit(1);
		}

		ROS_INFO("----------%s----------", matchesPCD[i].c_str());
		ROS_INFO("File label is %s", label.c_str());
		// create a new cloud each time to get a different pointer.
		typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
		reader.read(matchesPCD[i], *cloud);
		ROS_INFO("Cloud size is %d", (int)cloud->size());
		clouds.push_back(AnnotatedCloud<PointT>(label, matchesPCD[i], cloud));
	    }
    
	    return clouds;
	}

	// define the instantiations that we know we need
	template std::vector<AnnotatedCloud<pcl::PointXYZRGB> > getProcessedAnnotatedClouds(std::string filePath);
	template std::vector<AnnotatedCloud<pcl::PointXYZRGB> > getRawAnnotatedClouds(std::string filePath);
		
    } // namespace pclutil
} // namespace objsearch
