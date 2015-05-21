#ifndef OBJECT_QUERY_H
#define OBJECT_QUERY_H

#include "sysutil/sysutil.hpp"
#include "rosutil/rosutil.hpp"
#include "pclutil/annotationExtract.hpp"
#include "pclutil/cloudBounds.hpp"
#include "pclutil/cloudViewer.hpp"
#include "pclutil/pointValidation.hpp"
#include "pclutil/grid3d.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <limits>
#include <typeinfo>
#include <utility>
#include <vector>

#include <ros/console.h>

#include <pcl/common/common.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
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
	    struct QueryInfo {
		std::string fname;
		float queryTime;
		float houghTime;
		int pointsTotal;
		int pointsNonZero;
		int votesTotal;
		int pointsInBox;
		int votesInBox;
		int pointsMaxTotal;
		int pointsMaxInBox;
		int votesMaxInBox;
		int votesMaxTotal;
		std::string pointHistogram;
		std::string boxHistogram;
		std::string maxHistogram;
		std::string boxMaxHistogram;
	    };
	    
	    ObjectQuery(int argc, char *argv[]);

	    bool initAndCheckPaths(std::string path);
	    template<typename DescType>
	    void doSearch();
	    void annotatePointsCloud(
		std::string dir, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		std::vector<int>& indices, std::vector<std::string>& labels,
		std::vector<float>& distances, float maxDist);
	    void annotatePointsOBB(
		std::string dir, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		std::vector<int>& indices, std::vector<std::string>& labels,
		std::string queryLabel="NULL");
	    void postProcess(const pclutil::Grid3D& grid,
			     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& voteCloud,
			     const std::vector<int> cellIndices,
			     const std::vector<std::pair<int, int> >& maxPoints, QueryInfo info);
	    pclutil::Grid3D houghVoting(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& targetPoints,
			     const std::vector<std::vector<int> >& indices,
			     const std::vector<std::vector<float> >& distances);
	private:
	    std::string queryFile_;
	    std::string targetFile_;
	    std::string queryPointFile_;
	    std::string targetPointFile_;
	    std::string dataPath_;
	    std::string dataSubDir_;
	    std::string outDir_;
	    std::string outPath_;
	    std::string queryType_;
	    std::string queryObjectLabel_;

	    float xStepHough_;
	    float yStepHough_;
	    float zStepHough_;
	    int nMax_;
	    float clusterTolerance_;
	    float clusterMinSize_;
	    float clusterMaxSize_;

	    std::vector<std::string> targetClouds_;
	    int K_; // number of nearest neighbours to find

	    bool outputRegions_; // unused

	    std::string vectorToString(std::vector<int> vec);
	};
	
    } // namespace objectquery
} // namespace objsearch

#endif // OBJECT_QUERY_H




