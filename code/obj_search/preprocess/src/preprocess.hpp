/**
 * @file   planeTest.hpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Tue Mar 17 13:18:58 2015
 * 
 * @brief  
 * 
 * 
 */
#ifndef PREPROCESS_H

#define PREPROCESS_H

#include "rosutil/rosutil.hpp"
#include "sysutil/sysutil.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/transforms.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <simple_xml_parser.h>

namespace objsearch {
    namespace preprocessing {
	
	/** 
	 * @class PreprocessRoom 
	 * @brief Class for storing attributes of the preprocessing node
	 */
	class PreprocessRoom {
	public:
	    PreprocessRoom(int argc, char* argv[]);
            void loadRoom(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
	                  tf::StampedTransform& roomTransform, std::string fileXMLPath);
            void transformAndRemoveFloorCeiling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
				                const tf::StampedTransform& roomTransform);
	    void extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
	private:
	    std::string cloudDir; // directory containing the target cloud
	    std::string roomXML; // xml file with information about the room and intermediate clouds
	    std::string roomCloud; // the file containing the merged cloud
	    std::string dataPath; // top level directory which contains the data
	    std::string dataSubDir; // sub directory within the data path containing the cloud we are interested in
	    std::string outDir; // directory to which processed clouds will be output
	    std::string outPath;
	    
	    float ransacDistanceThresh;
	    int ransacIterations;
	    int planesToExtract;
	    float floorOffset;
	    float ceilingOffset;
	    float floorZ;
	    float ceilingZ;
	};
    } // namespace preprocessing
} // namespace objsearch

#endif // PREPROCESS_H




