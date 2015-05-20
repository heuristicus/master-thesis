#ifndef GRID_3D_HPP
#define GRID_3D_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include "colourConversion.hpp"
#include "sysutil/sysutil.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace objsearch {
    namespace pclutil {
	class Grid3D {
	public:
	    Grid3D(float _dim, float _step);
	    Grid3D(float _xDim, float _yDim, float _zDim,
		   float _xStep, float _yStep, float _zStep,
		   float _xOffset, float _yOffset, float _zOffset);

	    // should probably be private
	    float xDim_, yDim_, zDim_; // size of each dimension
	    float xStep_, yStep_, zStep_; // step in each dimension - defines cell dimensions
	    float xOffset_, yOffset_, zOffset_;
	    int width_, height_, depth_; 
	    std::vector<int> values_; // store value in each cell
	    
	    int indexFromDimIndices(int x, int y, int z);
	    pcl::PointXYZ cellCentreFromIndex(int index);
	    pcl::PointXYZ cellCentreFromIndex(int x, int y, int z);
	    pcl::PointXYZ cellCentreFromPoint(const pcl::PointXYZ& point);
	    pcl::PointXYZ cellCentreFromPoint(float x, float y, float z);
	    std::vector<pcl::PointXYZ> allCentres();
	    int& at(int index);
	    int& at(float x, float y, float z);
	    int& at(const pcl::PointXYZ& point);
	    int pointIndex(float x, float y, float z);
	    int pointIndex(const pcl::PointXYZ& point);
	    void indexUnflatten(int index, int& x, int& y, int& z);
	    std::pair<pcl::PointXYZ, int> getMax();
	    std::vector<std::pair<pcl::PointXYZ, int> > getMaxN(int n);
	    std::pair<pcl::PointXYZ, int> getMin();
	    int getValuesTotal();
	    int getEmptyTotal();
	    size_t size();
	    void toPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
	    
	};
    } // namespace pclutil
} // namespace objsearch

#endif // GRID_3D_HPP
