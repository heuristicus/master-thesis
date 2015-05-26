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
#include <pcl/filters/passthrough.h>
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
	    struct ClusterInfo {
		ClusterInfo(){score = 0;}
		int score; // number of votes this cluster got
		int points;
		int index; // initial index of the cluster
		pcl::PointXYZ centroid; // centroid of the cluster
		std::string parentCloud; // cloud file from which the region was extracted
		std::string regionCloud; // cloud file of the region
		std::string clusterCloud; // cloud file of the cluster
	    };

	    
	    struct QueryInfo {
		QueryInfo(){
		    fname = "-"; queryTime = -1; houghTime = -1; clusterTime = -1;
		    nClusters = -1; clusterScores = "-"; clusterPoints = "-";
		    clusterCentroidInOBB = "-"; pointsTotal = -1; pointsNonZero = -1;
		    votesTotal = -1; pointsInBox = -1; votesInBox = -1; pointsMaxTotal = -1;
		    votesMaxTotal = -1; pointsMaxInBox = -1; votesMaxInBox = -1; pointHistogram = "-";
		    boxHistogram = "-"; maxHistogram = "-"; boxMaxHistogram = "-"; topCluster = ClusterInfo();
		}
		std::string fname;
		// \brief time taken to find NNs for all the points in the query cloud
		float queryTime;
		// \brief time taken to compute the hough voting
		float houghTime;
		// \brief time take to compute clusters
		float clusterTime;
		// \brief number of clusters extracted from the point cloud
		// constructed from max n values in the hough vote cloud
		int nClusters;
		// \brief score of each cluster
		std::string clusterScores;
		// \brief points in each cluster
		std::string clusterPoints;
		// \brief is the cluster centroid inside the OBB of the actual
		// object for that cloud (if known)
 		std::string clusterCentroidInOBB;
		// \brief total number of points in the hough vote cloud
		int pointsTotal;
		// \brief total points with nonzero values
 		int pointsNonZero;
		// \brief total votes cast
		int votesTotal;
		// \brief total points wih votes that fall into the OBB of the object
		int pointsInBox;
		// \brief total votes contributed by the points that fall within the OBB
		int votesInBox;
		// \brief total number of points extracted from a sorted list of votes
		int pointsMaxTotal;
		// \brief total votes contributed by the max points
		int votesMaxTotal;
		// \brief total number of the highest valued points that fall in the OBB
		int pointsMaxInBox;
		// \brief total votes contributed by max points that are in the OBB
		int votesMaxInBox;
		// \brief histogram of values in the entire grid
		std::string pointHistogram;
		// \brief histogram of values falling inside the OBB
		std::string boxHistogram;
		// \brief histogram of all the highest valued points
		std::string maxHistogram;
		// \brief histogram of all the highest valued points which are in the OBB
 		std::string boxMaxHistogram;

		ClusterInfo topCluster;
	    };

	    ObjectQuery(int argc, char *argv[]);

	    void extractFeatureFileInfo(const std::string& fname,
					std::string& dateTime,
					std::string& interestType,
					std::string& featureType,
					std::string& originalName);
	    bool initAndCheckPaths(std::string path);
	    template<typename DescType>
	    void doSearch();
	    void annotatePointsCloud(
		std::string dir, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		std::vector<int>& indices, std::vector<std::string>& labels,
		std::vector<float>& distances, float maxDist);
	    std::vector<pclutil::OrientedBoundingBox> annotatePointsOBB(
		std::string dir, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		std::vector<int>& indices, std::vector<std::string>& labels,
		std::string queryLabel="NULL");
	    void postProcess(const pclutil::Grid3D& grid,
			     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& voteCloud,
			     const std::vector<int> cellIndices,
			     const std::vector<std::pair<int, int> >& maxPoints,
			     QueryInfo& info, std::vector<ClusterInfo> clusterDetails);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractClusterRegionBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
									const pcl::PointXYZ& centroid,
									const pcl::PointXYZ& extents);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractClusterRegionSphere(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const pcl::PointXYZ& centre, float radius);

	    pclutil::Grid3D houghVoting(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& targetPoints,
					const std::vector<std::vector<int> >& indices,
					const std::vector<std::vector<float> >& distances);
	    void writeInfo(std::string outFile, const std::vector<QueryInfo>& infoVec);
	private:
	    std::string queryFile_;
	    std::string targetFile_;
	    std::string queryPointFile_;
	    std::string targetPointFile_;
	    std::string originalCloudFileName_;
	    std::string originalTargetCloudFile_; // the cloud from which target features were extracted
	    std::string resultsOut_;
	    std::string dataPath_;
	    std::string dataSubDir_;
	    std::string outDir_;
	    std::string outPath_;
	    std::string queryType_;
	    std::string queryObjectLabel_;
	    std::string interestType_;
	    std::string featureType_;
	    pclutil::OrientedBoundingBox queryBbox_;

	    float xStepHough_;
	    float yStepHough_;
	    float zStepHough_;
	    int nMax_;
	    float clusterTolerance_;
	    float clusterMinSize_;
	    float clusterMaxSize_;
	    float extractRadiusMult_;

	    std::vector<std::string> targetClouds_;
	    int K_; // number of nearest neighbours to find

	    bool outputRegions_; // unused
	    bool clustersToResults_;
	    int keepNSubDirs_;

	    std::string vectorToString(std::vector<int> vec);
	};

	std::ostream& operator<<(std::ostream& os, const ObjectQuery::ClusterInfo& info) {
	    os << "ClusterInfo: score=" << info.score << ", points=" << info.points
	       << "centroid=" << info.centroid << ", parent=" << info.parentCloud
	       << "region=" << info.regionCloud << ", cluster=" << info.clusterCloud;
	    return os;
	}
	
    } // namespace objectquery
} // namespace objsearch

#endif // OBJECT_QUERY_H
