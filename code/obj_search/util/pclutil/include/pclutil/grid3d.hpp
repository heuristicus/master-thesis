#ifndef GRID_3D_HPP
#define GRID_3D_HPP

#include <cmath>
#include <vector>

#include <pcl/point_types.h>

namespace objsearch {
    namespace pclutil {
	class Grid3D {
	public:
	    Grid3D(float _dim, float _step);
	    Grid3D(float _xDim, float _yDim, float _zDim, float _xStep, float _yStep, float _zStep);

	    float xDim_, yDim_, zDim_; // size of each dimension
	    float xStep_, yStep_, zStep_; // step in each dimension - defines cell dimensions
	    int width_, height_, depth_; 
	    std::vector<int> values_; // store value in each cell

	    int indexFromDimIndices(int x, int y, int z);
	    pcl::PointXYZ cellCentre(float x, float y, float z);
	    pcl::PointXYZ cellCentre(const pcl::PointXYZ& point);
	    std::vector<pcl::PointXYZ> allCentres();
	    int& at(int index);
	    int& at(float x, float y, float z);
	    int& at(const pcl::PointXYZ& point);
	    int pointIndex(float x, float y, float z);
	    int pointIndex(const pcl::PointXYZ& point);
	    void indexUnflatten(int index, int& x, int& y, int& z);
	};
    } // namespace pclutil
} // namespace objsearch

#endif // GRID_3D_HPP
