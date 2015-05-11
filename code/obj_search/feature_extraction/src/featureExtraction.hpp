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
	    template<typename DescType, typename PointType>
	    void writeData(const typename pcl::PointCloud<DescType>::Ptr& descriptors,
			   const typename pcl::PointCloud<PointType>::Ptr& points);
	    void initPaths(std::string path);
	    void writeInfo(std::string outPath, FeatureInfo info, bool append);
	    void loadNormals(pcl::PointCloud<pcl::Normal>::Ptr& normals);
	    
	    // strings to store information about directories and the like for
	    // saving and retrieving data.
	    std::string dataSubDir_;
	    std::string outDir_;
	    std::string rawDir_;
	    std::string processedDir_;
	    std::string cloudFile_;
	    std::string outPath_;
	    std::string featureType_;
	    std::string featureSelection_;

	    float downsampleLeafSize_;

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
	};

    } // namespace feature_extraction
} // namespace objsearch

#endif // FEATURE_EXTRACTION_H
