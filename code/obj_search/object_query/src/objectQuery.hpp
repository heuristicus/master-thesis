#ifndef OBJECT_QUERY_H
#define OBJECT_QUERY_H

#include "sysutil/sysutil.hpp"
#include "rosutil/rosutil.hpp"
#include "pclutil/annotationExtract.hpp"
#include "pclutil/cloudBounds.hpp"
#include "pclutil/cloudViewer.hpp"
#include "pclutil/pointValidation.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <limits>
#include <typeinfo>
#include <utility>
#include <vector>

#include <ros/console.h>

#include <pcl/features/shot.h>
#include <pcl/features/usc.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/search/pcl_search.h>

namespace objsearch {
    namespace objectquery {

	class ObjectQuery {
	public:
	    ObjectQuery(int argc, char *argv[]);
	    
	    template<typename DescType>
	    void doSearch();
	    void annotatePointsCloud(
		std::string dir, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		std::vector<int>& indices, std::vector<std::string>& labels,
		std::vector<float>& distances, float maxDist);
	    void annotatePointsOBB(
		std::string dir, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		std::vector<int>& indices, std::vector<std::string>& labels);
	private:
	    std::string queryFile_;
	    std::string targetFile_;
	    std::string queryPointFile_;
	    std::string targetPointFile_;
	    std::string dataPath_;
	    std::string dataSubDir_;
	    std::string outDir_;
	    std::string outPath_;
	    int K_; // number of nearest neighbours to find

	    bool outputRegions_;
	};
	
    } // namespace objectquery
} // namespace objsearch

#endif // OBJECT_QUERY_H




