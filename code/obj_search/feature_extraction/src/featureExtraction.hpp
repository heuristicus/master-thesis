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
#include <vector>

#include <ros/console.h>

#include <pcl/common/common.h>
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
	    FeatureExtractor(int argc, char *argv[]);

	    void extractFeatures();
	    template<typename DescType, typename PointType>
	    void writeData(const typename pcl::PointCloud<DescType>::Ptr& descriptors,
			   const typename pcl::PointCloud<PointType>::Ptr& points);

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
	};

    } // namespace feature_extraction
} // namespace objsearch

#endif // FEATURE_EXTRACTION_H
