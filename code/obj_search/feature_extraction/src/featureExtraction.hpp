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

#include <pcl/features/shot.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/features/usc.h>
#include <pcl/filters/voxel_grid.h>

namespace objsearch {
    namespace featureExtraction {
	class FeatureExtractor {
	public:
	    FeatureExtractor(int argc, char *argv[]);

	    void extractFeatures();

	    // strings to store information about directories and the like for
	    // saving and retrieving data.
	    std::string dataSubDir;
	    std::string outDir;
	    std::string rawDir;
	    std::string processedDir;
	    std::string cloudFile;
	    std::string outPath;
	    std::string featureType;
	};

    } // namespace feature_extraction
} // namespace objsearch

#endif // FEATURE_EXTRACTION_H
