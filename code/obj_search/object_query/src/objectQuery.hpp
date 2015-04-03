#ifndef OBJECT_QUERY_H
#define OBJECT_QUERY_H

#include "sysutil/sysutil.hpp"
#include "rosutil/rosutil.hpp"

#include <cmath>
#include <string>
#include <typeinfo>
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
	private:
	    std::string queryFile_;
	    std::string targetFile_;
	    std::string dataPath_;
	    std::string dataSubDir_;
	    std::string outDir_;
	    std::string outPath_;
	    int K_; // number of nearest neighbours to find
	};
	
    } // namespace objectquery
} // namespace objsearch

#endif // OBJECT_QUERY_H




