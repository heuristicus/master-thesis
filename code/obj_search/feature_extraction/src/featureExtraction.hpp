/**
 * @file   featureExtraction.hpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Thu Apr  2 11:30:24 2015
 * 
 * @brief  
 * 
 * 
 */

#ifndef FEATURE_EXTRACTION_H
#define FEATURE_EXTRACTION_H

#include "sysutil/sysutil.hpp"
#include "rosutil/rosutil.hpp"
#include "pclutil/colourConversion.hpp"

#include <algorithm>
#include <string>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <iostream>

#include <ros/console.h>

#include <pcl/common/common.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/usc.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/susan.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>

namespace objsearch {
    namespace featureExtraction {
	class FeatureExtractor {
	public:
	    struct FeatureInfo {
		std::string fname;
		int originalSize;
		int featureSize; // number of points at which features will be computed
		double selectTime; // time to compute computation locations
		double featureTime; // time to compute features
	    };
	    FeatureExtractor(int argc, char *argv[]);

	    FeatureInfo extractFeatures();
	    void getDescriptorLocations(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr& descriptorLocations,
					FeatureInfo& info);

	    template<typename PointType>
	    void writeDescriptorLocs(const typename pcl::PointCloud<PointType>::Ptr& points);
	    template<typename DescType>
	    void writeDescriptors(const typename pcl::PointCloud<DescType>::Ptr& descriptors);
	    void writeInfo(std::string outPath, FeatureInfo info, bool first);
	    void loadNormals(pcl::PointCloud<pcl::Normal>::Ptr& normals);
	    void initPaths(std::string path);
	    std::string makeDescriptorLocationFileName();
	    
	    // strings to store information about directories and the like for
	    // saving and retrieving data.
	    std::string dataSubDir_;
	    std::string outDir_;
	    std::string rawDir_;
	    std::string processedDir_;
	    std::string cloudFile_;
	    std::string outPath_;
	    std::string featureType_;
	    std::string interestType_;
	    std::string dateTime_;
	    int cloudOffset_;

	    // Uniform sampling
	    float downsampleLeafSize_;

	    // ISS parameters
	    float issSalientMult_;
	    float issNonMaxMult_;
	    float issMinNeighbours_;
	    float issThreshold21_;
	    float issThreshold32_;

	    // SUSAN parameters
	    bool susanNonMax_;
	    float susanRadius_;
	    float susanDistThresh_;
	    float susanAngularThresh_;
	    float susanIntensityThresh_;

	    // Harris3d parameters
	    bool harrisNonMax_;
	    bool harrisRefine_;
	    float harrisThreshold_;
	    float harrisRadius_;

	    // SIFT parameters
	    float siftMinScale_;
	    float siftMinContrast_;
	    int siftOctaves_;
	    int siftOctaveScales_;

	    // Standard SHOT parameters
	    float shotRadius_;

	    // SHOTCOLOR parameters

	    // USC parameters
	    float uscRadius_;
	    float uscMinRadius_;
	    float uscDensityRadius_;
	    float uscLocalRadius_;

	    // fpfh parameters
	    float fpfhRadius_;

	    // pfh
	    float pfhRadius_;
	    
	    // pfhrgb
	    float pfhrgbRadius_;

	    bool doFeatures_;
	};

    } // namespace feature_extraction
} // namespace objsearch

#endif // FEATURE_EXTRACTION_H
