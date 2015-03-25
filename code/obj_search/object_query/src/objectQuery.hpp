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
	    std::string queryFile;
	    std::string targetFile;
	    std::string dataPath;
	    std::string dataSubDir;
	    std::string outDir;
	    std::string outPath;
	};
	
    } // namespace objectquery
} // namespace objsearch

#endif // OBJECT_QUERY_H




