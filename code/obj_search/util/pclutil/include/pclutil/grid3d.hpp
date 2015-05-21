#ifndef GRID_3D_HPP
#define GRID_3D_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include "colourConversion.hpp"
#include "sysutil/sysutil.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>

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
	    
	    int indexFromDimIndices(int x, int y, int z) const;
	    pcl::PointXYZ cellCentreFromIndex(int index) const;
	    pcl::PointXYZ cellCentreFromIndex(int x, int y, int z) const;
	    pcl::PointXYZ cellCentreFromPoint(const pcl::PointXYZ& point) const;
	    pcl::PointXYZ cellCentreFromPoint(float x, float y, float z) const;
	    std::vector<pcl::PointXYZ> allCentres() const;
	    int& at(int index);
	    int& at(float x, float y, float z);
	    int& at(const pcl::PointXYZ& point);
	    int at(int index) const;
	    int at(float x, float y, float z) const;
	    int at(const pcl::PointXYZ& point) const;
	    int pointIndex(float x, float y, float z) const;
	    int pointIndex(const pcl::PointXYZ& point) const;
	    void indexUnflatten(int index, int& x, int& y, int& z) const;
	    std::vector<std::pair<int, int> > getRankedIndices() const;
	    std::vector<std::pair<int, int> > getIndices() const;
	    std::pair<int, int> getMax() const;
	    std::vector<std::pair<int, int> > getMaxN(int n) const;
	    std::pair<int, int> getMin() const;
	    int getValuesTotal() const;
	    int getEmptyTotal() const;
	    int getTotalAbove(int thresh) const;
	    size_t size() const;
	    std::vector<int> valueHistogram() const;
	    std::vector<int> toPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) const;
	    
	};
    } // namespace pclutil
} // namespace objsearch

#endif // GRID_3D_HPP
