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

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
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
            void preprocessCloud();
            void loadCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
			   tf::StampedTransform& cloudTransform);
            void transformAndRemoveFloorCeiling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
				                const tf::StampedTransform& cloudTransform);
	    void extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
			       pcl::PointCloud<pcl::Normal>::Ptr& normals);
	    template<typename SegmentationType>
	    void extractPlanes(SegmentationType& seg,
			       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
			       pcl::PointCloud<pcl::Normal>::Ptr& normals);
	    void computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
				pcl::PointCloud<pcl::Normal>::Ptr& normals,
				const tf::StampedTransform& cloudTransform);

	    // some constants to use when dealing with filenames and what needs
	    // to be done to properly process it
	    static const std::string intermediatePrefix;
	    static const std::string completeCloudName;
	private:
	    enum class CloudType {
		INTERMEDIATE, FULL, OTHER
	    };
	    
	    std::string cloudPath; // path of the target cloud
	    std::string cloudDir;
	    std::string cloudFile; // path to the target cloud
	    std::string roomXML; // xml file with information about the room and intermediate clouds
	    std::string dataPath; // top level directory which contains the data
	    std::string dataSubDir; // sub directory within the data path containing the cloud we are interested in
	    std::string outDir; // directory to which processed clouds will be output
	    std::string outPrefix; // prefix for the output filename, used for intermediate clouds
	    std::string outPath;
	    // a string to determine what sort of cloud is being passed in; a
	    // full cloud, intermediate cloud, or some other random cloud
	    CloudType type;

	    int cloudNum; // the number of the intermediate cloud (or -1 if not working on an intermediate)
	    
	    // flags for executing different parts of preprocessing
	    bool doExtractPlanes;
	    bool doTrimCloud;
	    bool doComputeNormals;
	    bool doDownsample;

	    // plane extraction parameters
	    float ransacDistanceThresh;
	    float minPlanePropComplete;
	    float minPlanePropIntermediate;
	    int ransacIterations;
	    int planesToExtract;
	    int planeSkipLimit;
	    int minPlanePointsComplete;
	    int minPlanePointsIntermediate;

	    // trimming parameters
	    float floorOffset;
	    float ceilingOffset;
	    float floorZ;
	    float ceilingZ;

	    float normalRadius;
	    float downsampleLeafSize;
	    
	};
    } // namespace preprocessing
} // namespace objsearch

#endif // PREPROCESS_H




