#include "pclutil/grid3d.hpp"

namespace objsearch {
    namespace pclutil {

	/** 
	 * Simple constructor which creates a cube grid
	 * 
	 * @param _dim length of each dimension
	 * @param _step step for each cell
	 * 
	 * @return 
	 */
	Grid3D::Grid3D(float _dim, float _step) {
	    xDim_ = yDim_ = zDim_ = _dim;
	    xStep_ = yStep_ = zStep_ = _step;

	    // width, height and depth in terms of number of indices assigned to
	    // each need to round up to make sure that if the step does not go
	    // exactly into the dimensions then we do not miss parts of the
	    // space which would end up outside the basic division.
	    width_ = std::ceil(xDim_/xStep_);
	    height_ = std::ceil(zDim_/zStep_); 
	    depth_ = std::ceil(yDim_/yStep_);

	    // number of indices in the grid are all its dimensions multiplied
	    values_.resize(width_ * height_ * depth_);
	}

	/** 
	 * More complex constructor. Z is the height of the grid, y is its
	 * depth, and x its width.
	 *
	 *  z   y
	 *  ^  /|
	 *  |  /
	 *  | /
	 *  |/____> x
	 *
	 * The offsets specify the origin for the box, so that cells are in the
	 * correct region of space for the space that is being described.
	 * 
	 * @param _xDim Length of the x dimension
	 * @param _yDim y dimension
	 * @param _zDim z dimension
	 * @param _xStep Step of the x dimension
	 * @param _yStep y dim
	 * @param _zStep z dim
	 * @param _xOffset Offset of the x dimension.
	 * @param _yOffset y dim
	 * @param _zOffset z dim
	 * 
	 * @return 
	 */
	Grid3D::Grid3D(float _xDim, float _yDim, float _zDim,
		       float _xStep, float _yStep, float _zStep,
		       float _xOffset, float _yOffset, float _zOffset){
	    xDim_ = _xDim;
	    yDim_ = _yDim;
	    zDim_ = _zDim;
	    xStep_ = _xStep;
	    yStep_ = _yStep;
	    zStep_ = _zStep;
	    xOffset_ = _xOffset;
	    yOffset_ = _yOffset;
	    zOffset_ = _zOffset;

	    width_ = std::ceil(xDim_/xStep_);
	    height_ = std::ceil(zDim_/zStep_); 
	    depth_ = std::ceil(yDim_/yStep_);
	    
	    values_.resize(width_ * height_ * depth_);
	}

	/** 
	 * 
	 * 
	 * @param point 
	 * 
	 * @return 
	 */
	int Grid3D::pointIndex(const pcl::PointXYZ& point) const {
	    return pointIndex(point.x, point.y, point.z);
	}
	
	/** 
	 * Get the index in the array of the cell containing the point with the
	 * given coordinates.
	 * 
	 * @param x 
	 * @param y 
	 * @param z 
	 * 
	 * @return 
	 */
	int Grid3D::pointIndex(float x, float y, float z) const {
	    // integer truncation down to correct index. Make sure that the
	    // point is translated to the origin frame of reference to get
	    // correct computations
	    return indexFromDimIndices((x - xOffset_)/xStep_,
				       (y - yOffset_)/yStep_,
				       (z - zOffset_)/zStep_);
	}

	/** 
	 * Get the index of a cell in the grid based on the indices of
	 * individual dimensions (like indexing into a 3d array)
	 * 
	 * @param x 
	 * @param y 
	 * @param z 
	 * 
	 * @return 
	 */
	int Grid3D::indexFromDimIndices(int x, int y, int z) const {
	    // http://stackoverflow.com/questions/7367770/how-to-flatten-or-index-3d-array-in-1d-array
	    if (x >= width_ || y > depth_ || z > height_){
		std::string error("Dimension exceeds limit\nxmax = " + std::to_string(width_) + ", x = " + std::to_string(x)
				  + "\nymax = " + std::to_string(depth_) + ", y = " + std::to_string(y)
				  + "\nzmax = " + std::to_string(height_) + ", z = " + std::to_string(z));
		throw sysutil::objsearchexception(error);
	    }
	    return x + width_ * (y + z * depth_);
	}

	/** 
	 * Get the centre of the cell with the given index
	 * 
	 * @param ind 
	 * 
	 * @return 
	 */
	pcl::PointXYZ Grid3D::cellCentreFromIndex(int ind) const {
	    int x, y, z;
	    indexUnflatten(ind, x, y, z);
	    return cellCentreFromIndex(x, y, z);
	}

	/** 
	 * Get the centre of a cell specified by its 3d indices
	 * 
	 * @param x 
	 * @param y 
	 * @param z 
	 * 
	 * @return 
	 */
	pcl::PointXYZ Grid3D::cellCentreFromIndex(int x, int y, int z) const {
	    return pcl::PointXYZ(xStep_/2 + x * xStep_ + xOffset_,
				 yStep_/2 + y * yStep_ + yOffset_,
				 zStep_/2 + z * zStep_ + zOffset_);
	}

	/** 
	 * Get the centre of the cell which contains the given point
	 * 
	 * @param point 
	 * 
	 * @return 
	 */
	pcl::PointXYZ Grid3D::cellCentreFromPoint(const pcl::PointXYZ& point) const {
	    return cellCentreFromPoint(point.x, point.y, point.z);
	}

	/** 
	 * Get the centre of the cell containing the point specified by the three values
	 * 
	 * @param x 
	 * @param y 
	 * @param z 
	 * 
	 * @return 
	 */
	pcl::PointXYZ Grid3D::cellCentreFromPoint(float x, float y, float z) const {
	    pcl::PointXYZ point;
	    // the second part gives the minimum point in the cell on that axis,
	    // centre is where half the step for that axis is added to the offset
	    point.x = xOffset_ + xStep_/2 + std::floor((x - xOffset_)/xStep_) * xStep_;
	    point.y = yOffset_ + yStep_/2 + std::floor((y - yOffset_)/yStep_) * yStep_;
	    point.z = zOffset_ + zStep_/2 + std::floor((z - zOffset_)/zStep_) * zStep_;
	    return point;
	}

	/** 
	 * 
	 * 
	 * @return 
	 */
	std::vector<pcl::PointXYZ> Grid3D::allCentres() const {
	    std::vector<pcl::PointXYZ> centres;
	    for (size_t i = 0; i < values_.size(); i++) {
		centres.push_back(cellCentreFromIndex(i));
	    }
	    return centres;
	}
	
	/** 
	 * 
	 * 
	 * @param point 
	 * 
	 * @return 
	 */
	int& Grid3D::at(const pcl::PointXYZ& point) {
	    return at(point.x, point.y, point.z);
	}

	/** 
	 * Get the value of the grid cell with the given coordinates.
	 * 
	 * @param x 
	 * @param y 
	 * @param z 
	 * 
	 * @return Reference to the value of the cell
	 */
	int& Grid3D::at(float x, float y, float z) {
	    return at(pointIndex(x, y, z));
	}

	/** 
	 * 
	 * 
	 * @param index 
	 * 
	 * @return 
	 */
	int& Grid3D::at(int index) {
	    return values_[index];
	}

	/** 
	 * 
	 * 
	 * @param point 
	 * 
	 * @return 
	 */
	int Grid3D::at(const pcl::PointXYZ& point) const {
	    return at(point.x, point.y, point.z);
	}

	/** 
	 * Get the value of the grid cell with the given coordinates.
	 * 
	 * @param x 
	 * @param y 
	 * @param z 
	 * 
	 * @return Reference to the value of the cell
	 */
	int Grid3D::at(float x, float y, float z) const {
	    return at(pointIndex(x, y, z));
	}

	/** 
	 * 
	 * 
	 * @param index 
	 * 
	 * @return 
	 */
	int Grid3D::at(int index) const {
	    return values_[index];
	}

	/** 
	 * Unflatten an index from 1D to 3D. e.g. values[26] = values[2][2][2]
	 * 
	 * @param index Index to unflatten
	 * @param x 3d x index
	 * @param y 3d y index
	 * @param z 3d z index
	 */
	void Grid3D::indexUnflatten(int index, int& x, int& y, int& z) const {
	    if (index >= (int)values_.size()) {
		std::string error("Index is out of bounds. Vector length: " + std::to_string(values_.size()) + " requested index: " + std::to_string(index));
		throw sysutil::objsearchexception(error);
	    }
	    z = std::floor(index / (width_ * depth_));
	    y = (int)std::floor(index / width_) % depth_;
	    x = index % width_;
	}

	/** 
	 * Get a vector of pairs of two ints, containing the index and value at
	 * that index
	 * 
	 * 
	 * @return Vector containing indices and values at those indices.
	 */
	std::vector<std::pair<int, int> > Grid3D::getIndices() const {
	    // populate a vector with the index and value at that index
	    std::vector<std::pair<int,int> > items;
	    for (size_t i = 0; i < values_.size(); i++) {
		items.push_back(std::pair<int, int>(i, values_[i]));
	    }
	    return items;
	}

	/** 
	 * Get a vector containing all of the values in the grid, sorted in
	 * descending order.
	 * 
	 * 
	 * @return Vector of pairs of two ints, first is the index of the point,
	 * second is the value. Sorted in descending order of value.
	 */
	std::vector<std::pair<int, int> > Grid3D::getRankedIndices() const {
	    auto comp = [](std::pair<int, int> p, std::pair<int, int> q){
		return p.second > q.second;
	    };
	    
	    // populate a vector with the index and value at that index
	    std::vector<std::pair<int,int> > items = getIndices();
	    std::sort(items.begin(), items.end(), comp);
	    return items;
	}

	/** 
	 * Get the centre of the cell with the maximum value, and the count in
	 * that cell.
	 * 
	 * @return 
	 */
	std::pair<int, int> Grid3D::getMax() const {
	    auto it = std::max_element(values_.begin(), values_.end());

	    // compute the index of the max point by subtracting the two
	    // iterators
	    return std::pair<int, int>(values_.end() - it, *it);
	}

	/** 
	 * Get the top \p n values in the values vector
	 * 
	 * @param n 
	 * 
	 * @return vector of pairs with the index of the cell and the count in
	 * that cell
	 */
	std::vector<std::pair<int, int> > Grid3D::getMaxN(int n) const {
	    auto comp = [](std::pair<int, int> p, std::pair<int, int> q){
		return p.second > q.second;
	    };

	    // make sure that n does not overflow
	    n = n > values_.size() ? values_.size() : n;
	    
	    // populate a vector with the index and value at that index
	    std::vector<std::pair<int,int> > items = getIndices();
	    // partially sort the vector so that the first n elements are sorted
	    // and the rest are in arbitrary order.
	    std::partial_sort(items.begin(), items.begin() + n, items.end(), comp);
	    return std::vector<std::pair<int,int> >(items.begin(), items.begin() + n);
	}

	/** 
	 * Get the centre of the cell with the minimum value, and the count in
	 * that cell.
	 * 
	 * @return 
	 */
	std::pair<int, int> Grid3D::getMin() const {
	    auto it = std::min_element(values_.begin(), values_.end());
	    
	    return std::pair<int, int>(values_.end() - it, *it);
	}

	/** 
	 * Get the sum of all values in the grid
	 * 
	 * @return 
	 */
	int Grid3D::getValuesTotal() const {
	    return std::accumulate(values_.begin(), values_.end(), 0);
	}

	/** 
	 * Get the total number of cells in the grid which have value zero
	 * 
	 * @return 
	 */
	int Grid3D::getEmptyTotal() const {
	    int empty = 0;
	    for (auto it = values_.begin(); it != values_.end(); it++) {
		if (*it == 0) {
		    empty++;
		}
	    }

	    return empty;
	}

	/** 
	 * Get the total number of cells with value above the given threshold
	 * 
	 * @param thresh threshold to apply
	 * 
	 * @return Number of cells above the threshold
	 */
	int Grid3D::getTotalAbove(int thresh) const {
	    int count = 0;
	    for (size_t i = 0; i < values_.size(); i++) {
		if (values_[i] > thresh) {
		    count++;
		}
	    }
	    return count;
	}

	/** 
	 * Get a histogram of the values contained by the grid
	 * 
	 * 
	 * @return vector with each index containing the number of points with
	 * the value of that index, i.e. index 0 contains the number of points
	 * with 0 votes, and so on.
	 */
	std::vector<int> Grid3D::valueHistogram() const {
	    std::vector<int> histogram(20);
	    int maxVotes = -1;
	    for (size_t i = 0; i < values_.size(); i++) {
		int curVal = values_[i];
		if (curVal > histogram.size() - 1) {
		    histogram.resize(curVal + 1);
		}
		if (curVal > maxVotes) {
		    maxVotes = curVal;
		}
		histogram[curVal]++;
	    }
	    // resize the histogram at the end to remove extra space
	    histogram.resize(maxVotes + 1);
	    return histogram;
	}

	/** 
	 * The size of the grid in terms of number of cells.
	 * 
	 * @return 
	 */
	size_t Grid3D::size() const {
	    return values_.size();
	}

	/** 
	 * Output this grid to a pointcloud. Each cell in the grid will be
	 * represented by its centre, and will be coloured according to the
	 * value inside that cell. Cells with high values are red, low values
	 * are blue.
	 * 
	 * @param cloud will be populated with points corresponding to cells
	 * with nonzero values. The points are coloured according to their value
	 * @return vector of indices in the grid corresponding to point indices in the cloud
	 */
	std::vector<int> Grid3D::toPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) const {
	     std::vector<pcl::PointXYZ> centres = allCentres();
	     std::vector<int> inds;
	     int maxVal = getMax().second;
	     // centres are generated using the indices of cells in the grid, so
	     // loop indices correspond to cell indices
	     for (size_t i = 0; i < centres.size(); i++) {
		 int value = at(i);
		 if (value == 0) {
		     continue;
		 }
		 pcl::PointXYZRGB np;
		 np.x = centres[i].x;
		 np.y = centres[i].y;
		 np.z = centres[i].z;
		 rgb colour = getHeatColour(at(i), maxVal);
		 np.r = colour.r * 255;
		 np.g = colour.g * 255;
		 np.b = colour.b * 255;
		 cloud->push_back(np);
		 inds.push_back(i);
	     }
	     return inds;
	 }

    } // namespace pclutil
} // namespace objsearch
