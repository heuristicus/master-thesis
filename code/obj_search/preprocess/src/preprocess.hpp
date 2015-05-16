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
#include "pclutil/annotationExtract.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <cmath>
#include <regex>

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
	    // store information about how long things take, number of points and the like
	    struct ProcessInfo {
		std::string fname;
		int originalPoints;
		int downsampledPoints;
		int trimmedPoints;
		int nonPlanePoints;
		double loadTime;
		double downsampleTime;
		double trimTime;
		double normalTime;
		double featureNormalTime;
		int numPlanes;
		double planeTime;
	    };
	    
	    PreprocessRoom(int argc, char* argv[]);
	    void initPaths(std::string path);
	    void writeInfo(std::string outPath, ProcessInfo info, bool append=false);
            ProcessInfo preprocessCloud();
	    void loadCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
			   tf::StampedTransform& cloudTransform,
			   tf::StampedTransform& registeredTransform);
	    void transformAndRemoveFloorCeiling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
						const tf::StampedTransform& cloudTransform,
						const tf::StampedTransform& registeredTransform);
	    int extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
			       pcl::PointCloud<pcl::Normal>::Ptr& normals);
	    template<typename SegmentationType>
	    int extractPlanes(SegmentationType& seg,
			       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
			       pcl::PointCloud<pcl::Normal>::Ptr& normals);
	    void computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
				pcl::PointCloud<pcl::Normal>::Ptr& normals,
				const tf::StampedTransform& cloudTransform,
				float radius);

	    // some constants to use when dealing with filenames and what needs
	    // to be done to properly process it
	    static const std::string intermediatePrefix;
	    static const std::string completeCloudName;
	private:
	    enum class CloudType {
		INTERMEDIATE, FULL, OTHER
	    };

	    std::string cloudPath_; // path of the target cloud
	    std::string cloudDir_;
	    std::string cloudFile_; // path to the target cloud
	    std::string roomXML_; // xml file with information about the room and intermediate clouds
	    std::string dataPath_; // top level directory which contains the data
	    std::string dataSubDir_; // sub directory within the data path containing the cloud we are interested in
	    std::string outDir_; // directory to which processed clouds will be output
	    std::string outPrefix_; // prefix for the output filename, used for intermediate clouds
	    std::string outPath_;

	    // a string to determine what sort of cloud is being passed in; a
	    // full cloud, intermediate cloud, or some other random cloud
	    CloudType type_;

	    int cloudNum_; // the number of the intermediate cloud (or -1 if not working on an intermediate)
	    int cloudOffset_;

	    // flags for executing different parts of preprocessing
	    bool doExtractPlanes_;
	    bool doTrimCloud_;
	    bool doRotateAnnotations_;
	    bool doComputeNormals_;
	    bool doDownsample_;
            bool savePlanes_; // save individual planes or just the combined cloud
	    // additional filtering on x and y dimensions after the transform is applied
	    bool filterX_;
	    bool filterY_;
	    

	    // plane extraction parameters
	    float ransacDistanceThresh_;
	    float minPlanePropComplete_;
	    float minPlanePropIntermediate_;
	    int ransacIterations_;
	    int planesToExtract_;
	    int planeSkipLimit_;
	    int minPlanePointsComplete_;
	    int minPlanePointsIntermediate_;

	    // trimming parameters
	    float floorOffset_;
	    float ceilingOffset_;
	    float floorZ_;
	    float ceilingZ_;

	    // additional filter parameters
	    float xFilterMax_;
	    float xFilterMin_;
	    float yFilterMax_;
	    float yFilterMin_;

	    float normalRadiusPlane_;
	    float normalRadiusFeature_;
	    float downsampleLeafSize_;
	    float downsampleIncrement_;
	    
	};
    } // namespace preprocessing
} // namespace objsearch

#endif // PREPROCESS_H
