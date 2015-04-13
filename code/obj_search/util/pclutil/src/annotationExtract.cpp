#include "pclutil/annotationExtract.hpp"

/** 
 * Get a vector of point clouds with their corresponding labels from the given
 * directory. Each cloud and label should have "label" somewhere in the
 * filename, and sorting the files should mean that when paired the correct
 * label .txt file is paired with its .pcd file.
 * 
 * @param filePath Path to the directory containing the clouds and their labels.
 * 
 * @return 
 */
template <typename PointT>
std::vector<AnnotatedCloud<PointT> > getAnnotatedClouds(std::string filePath) {
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
	clouds.push_back(AnnotatedCloud<PointT>(label, cloud));
    }
    
    return clouds;
}

int main(int argc, char *argv[]) {
    getAnnotatedClouds<pcl::PointXYZRGB>(argv[1]);
}
