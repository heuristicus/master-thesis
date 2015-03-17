/**
 * @file   planeTest.hpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Tue Mar 17 13:18:58 2015
 * 
 * @brief  
 * 
 * 
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "rosutil/rosutil.hpp"
#include "sysutil/sysutil.hpp"
#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <string>
#include <cstdlib>

namespace objsearch {
    namespace preprocessing {

	class PreprocessRoom {
	public:
	    PreprocessRoom(int argc, char* argv[]);
	    
	    void transformAndRemoveFloorCeiling();
	    void extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	private:
	    std::string cloudFile;
	    std::string dataPath;
	    std::string dataSubDir;
	    std::string outDir;
	    float ransacDistanceThresh;
	    int ransacIterations;
	    int planesToExtract;
	};
    } // namespace preprocessing
} // namespace objsearch
