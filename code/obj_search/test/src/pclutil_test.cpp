#include "pclutil/grid3d.hpp"
#include "pclutil/annotationExtract.hpp"
#include "pclutil/cloudBounds.hpp"
#include "pclutil/cloudViewer.hpp"
#include "pclutil/pointValidation.hpp"
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>

#include <gtest/gtest.h>

namespace testing {
    namespace pclutil {
	using namespace objsearch::pclutil;

	class PCLUtilTest : public ::testing::Test {
	protected:

	    PCLUtilTest() {
		// add to the vector in expected order of points
		// box has step of 1, so centres are at intervals of 0.5
		for (int z = 0; z < 5; z++) {
		    for (int y = 0; y < 3; y++) {
			for (int x = 0; x < 4; x++) {
			    boxCentres.push_back(pcl::PointXYZ(0.5 + x, 0.5 + y, 0.5 + z));
			}
		    }
		}

	    }
	    // cube and its corresponding expected centres for all cells
	    Grid3D cube3 = Grid3D(3, 3, 3, 1, 1, 1, 0, 0, 0);
	    // cells on the edges
	    pcl::PointXYZ frontBottom = pcl::PointXYZ(1.5, 0.5, 0.5);
	    pcl::PointXYZ frontTop = pcl::PointXYZ(1.5, 0.5, 2.5);
	    pcl::PointXYZ frontLeft = pcl::PointXYZ(0.5, 0.5, 1.5);
	    pcl::PointXYZ frontRight = pcl::PointXYZ(2.5, 0.5, 1.5);
	    pcl::PointXYZ backBottom = pcl::PointXYZ(1.5, 2.5, 0.5);
	    pcl::PointXYZ backTop = pcl::PointXYZ(1.5, 2.5, 2.5);
	    pcl::PointXYZ backLeft = pcl::PointXYZ(0.5, 2.5, 1.5);
	    pcl::PointXYZ backRight = pcl::PointXYZ(2.5, 2.5, 1.5);
	    pcl::PointXYZ midBottom = pcl::PointXYZ(1.5, 1.5, 0.5);
	    pcl::PointXYZ midTop = pcl::PointXYZ(1.5, 1.5, 2.5);
	    pcl::PointXYZ midLeft = pcl::PointXYZ(0.5, 1.5, 1.5);
	    pcl::PointXYZ midRight = pcl::PointXYZ(2.5, 1.5, 1.5);

	    // cells in the corners
	    pcl::PointXYZ frontBottomLeft = pcl::PointXYZ(0.5, 0.5, 0.5);
	    pcl::PointXYZ frontBottomRight = pcl::PointXYZ(2.5, 0.5, 0.5);
	    pcl::PointXYZ frontTopLeft = pcl::PointXYZ(0.5, 0.5, 2.5);
	    pcl::PointXYZ frontTopRight = pcl::PointXYZ(2.5, 0.5, 2.5);
	    pcl::PointXYZ backBottomLeft = pcl::PointXYZ(0.5, 2.5, 0.5);
	    pcl::PointXYZ backBottomRight = pcl::PointXYZ(2.5, 2.5, 0.5);
	    pcl::PointXYZ backTopLeft = pcl::PointXYZ(0.5, 2.5, 2.5);
	    pcl::PointXYZ backTopRight = pcl::PointXYZ(2.5, 2.5, 2.5);
	    pcl::PointXYZ midBottomLeft = pcl::PointXYZ(0.5, 1.5, 0.5);
	    pcl::PointXYZ midBottomRight = pcl::PointXYZ(2.5, 1.5, 0.5);
	    pcl::PointXYZ midTopLeft = pcl::PointXYZ(0.5, 1.5, 2.5);
	    pcl::PointXYZ midTopRight = pcl::PointXYZ(2.5, 1.5, 2.5);

	    // centre cells
	    pcl::PointXYZ frontCentre = pcl::PointXYZ(1.5, 0.5, 1.5);
	    pcl::PointXYZ backCentre = pcl::PointXYZ(1.5, 2.5, 1.5);
	    pcl::PointXYZ midCentre = pcl::PointXYZ(1.5, 1.5, 1.5);

	    Grid3D box = Grid3D(4, 3, 5, 1, 1, 1, 0, 0, 0);
	    std::vector<pcl::PointXYZ> boxCentres;
	};
	
	TEST_F(PCLUtilTest, grid3d_init_cube){
	    Grid3D grid(1.5, 0.5);
	    ASSERT_TRUE(grid.width_ == 3);
	    ASSERT_TRUE(grid.height_ == 3);
	    ASSERT_TRUE(grid.depth_ == 3);
	    ASSERT_TRUE(grid.values_.size() == 27);
	}

	TEST_F(PCLUtilTest, grid3d_init_box){
	    Grid3D grid(3, 2, 1, 1, 1, 1, 0, 0, 0);
	    ASSERT_TRUE(grid.width_ == 3);
	    ASSERT_TRUE(grid.height_ == 1);
	    ASSERT_TRUE(grid.depth_ == 2);
	    ASSERT_TRUE(grid.values_.size() == 6);
	}

	// make sure the initialisation works as expected when the dimension is
	// not exactly divisible by the step.
	TEST_F(PCLUtilTest, grid3d_init_box_overstep){
	    Grid3D grid(2, 2, 2, 0.6, 0.3, 0.7, 0, 0, 0);
	    ASSERT_TRUE(grid.width_ == 4);
	    ASSERT_TRUE(grid.height_ == 3);
	    ASSERT_TRUE(grid.depth_ == 7);
	    ASSERT_TRUE(grid.values_.size() == 4 * 3 * 7);
	}

	void pointEqual(const pcl::PointXYZ& p, const pcl::PointXYZ& q) {
	    ASSERT_FLOAT_EQ(p.x, q.x);
	    ASSERT_FLOAT_EQ(p.y, q.y);
	    ASSERT_FLOAT_EQ(p.z, q.z);
	}
	
	// Make sure the cell centre computation works for all corners of a simple cube
	TEST_F(PCLUtilTest, grid3d_cellcentre_corners){
	    pointEqual(frontBottomLeft, cube3.cellCentreFromPoint(0.1, 0.1, 0.1));
	    pointEqual(frontBottomRight, cube3.cellCentreFromPoint(2.1, 0.1, 0.1));
	    pointEqual(frontTopLeft, cube3.cellCentreFromPoint(0.1, 0.1, 2.1));
	    pointEqual(frontTopRight, cube3.cellCentreFromPoint(2.1, 0.1, 2.1));
	  
	    pointEqual(backBottomLeft, cube3.cellCentreFromPoint(0.1, 2.1, 0.1));
	    pointEqual(backBottomRight, cube3.cellCentreFromPoint(2.1, 2.1, 0.1));
	    pointEqual(backTopLeft, cube3.cellCentreFromPoint(0.1, 2.1, 2.1));
	    pointEqual(backTopRight, cube3.cellCentreFromPoint(2.1, 2.1, 2.1));

	    pointEqual(midBottomLeft, cube3.cellCentreFromPoint(0.1, 1.1, 0.1));
	    pointEqual(midBottomRight, cube3.cellCentreFromPoint(2.1, 1.1, 0.1));
	    pointEqual(midTopLeft, cube3.cellCentreFromPoint(0.1, 1.1, 2.1));
	    pointEqual(midTopRight, cube3.cellCentreFromPoint(2.1, 1.1, 2.1));
	}

	// Make sure the cell centre computation works for all edges of a simple cube
	TEST_F(PCLUtilTest, grid3d_cellcentre_edges) {
	    pointEqual(frontBottom, cube3.cellCentreFromPoint(1.1, 0.1, 0.1));
	    pointEqual(frontTop, cube3.cellCentreFromPoint(1.1, 0.1, 2.1));
	    pointEqual(frontLeft, cube3.cellCentreFromPoint(0.1, 0.1, 1.1));
	    pointEqual(frontRight, cube3.cellCentreFromPoint(2.1, 0.1, 1.1));
	  
	    pointEqual(backBottom, cube3.cellCentreFromPoint(1.1, 2.1, 0.1));
	    pointEqual(backTop, cube3.cellCentreFromPoint(1.1, 2.1, 2.1));
	    pointEqual(backLeft, cube3.cellCentreFromPoint(0.1, 2.1, 1.1));
	    pointEqual(backRight, cube3.cellCentreFromPoint(2.1, 2.1, 1.1));

	    pointEqual(midBottom, cube3.cellCentreFromPoint(1.1, 1.1, 0.1));
	    pointEqual(midTop, cube3.cellCentreFromPoint(1.1, 1.1, 2.1));
	    pointEqual(midLeft, cube3.cellCentreFromPoint(0.1, 1.1, 1.1));
	    pointEqual(midRight, cube3.cellCentreFromPoint(2.1, 1.1, 1.1));
	}
	
	// Make sure the cell centre computation works for the centre of a simple cube
	TEST_F(PCLUtilTest, grid3d_cellcentre_centre) {
	    pointEqual(frontCentre, cube3.cellCentreFromPoint(1.1, 0.1, 1.1));
	    pointEqual(backCentre, cube3.cellCentreFromPoint(1.1, 2.1, 1.1));
	    pointEqual(midCentre, cube3.cellCentreFromPoint(1.1, 1.1, 1.1));
	}

	TEST_F(PCLUtilTest, grid3d_allcentres_box) {
	    std::vector<pcl::PointXYZ> centres = box.allCentres();
	    for (size_t i = 0; i < centres.size(); i++) {
		pointEqual(centres[i], boxCentres[i]);
	    }
	}

	TEST_F(PCLUtilTest, grid3d_pointindex) {
	    // Bottom slice
	    ASSERT_EQ(0, cube3.pointIndex(frontBottomLeft));
	    ASSERT_EQ(1, cube3.pointIndex(frontBottom));
	    ASSERT_EQ(2, cube3.pointIndex(frontBottomRight));
	    ASSERT_EQ(3, cube3.pointIndex(midBottomLeft));
	    ASSERT_EQ(4, cube3.pointIndex(midBottom));
	    ASSERT_EQ(5, cube3.pointIndex(midBottomRight));
	    ASSERT_EQ(6, cube3.pointIndex(backBottomLeft));
	    ASSERT_EQ(7, cube3.pointIndex(backBottom));
	    ASSERT_EQ(8, cube3.pointIndex(backBottomRight));

	    // Middle slice
	    ASSERT_EQ(9, cube3.pointIndex(frontLeft));
	    ASSERT_EQ(10, cube3.pointIndex(frontCentre));
	    ASSERT_EQ(11, cube3.pointIndex(frontRight));
	    ASSERT_EQ(12, cube3.pointIndex(midLeft));
	    ASSERT_EQ(13, cube3.pointIndex(midCentre));
	    ASSERT_EQ(14, cube3.pointIndex(midRight));
	    ASSERT_EQ(15, cube3.pointIndex(backLeft));
	    ASSERT_EQ(16, cube3.pointIndex(backCentre));
	    ASSERT_EQ(17, cube3.pointIndex(backRight));

	    // Top slice
	    ASSERT_EQ(18, cube3.pointIndex(frontTopLeft));
	    ASSERT_EQ(19, cube3.pointIndex(frontTop));
	    ASSERT_EQ(20, cube3.pointIndex(frontTopRight));
	    ASSERT_EQ(21, cube3.pointIndex(midTopLeft));
	    ASSERT_EQ(22, cube3.pointIndex(midTop));
	    ASSERT_EQ(23, cube3.pointIndex(midTopRight));
	    ASSERT_EQ(24, cube3.pointIndex(backTopLeft));
	    ASSERT_EQ(25, cube3.pointIndex(backTop));
	    ASSERT_EQ(26, cube3.pointIndex(backTopRight));
	}

	TEST_F(PCLUtilTest, grid3d_cellcentre_box){
	    for (size_t i = 0; i < boxCentres.size(); i++) {
		pointEqual(boxCentres[i], box.cellCentreFromPoint(boxCentres[i]));
	    }
	}

	TEST_F(PCLUtilTest, grid3d_pointindex_box) {
	    for (size_t i = 0; i < boxCentres.size(); i++) {
		ASSERT_EQ(i, box.pointIndex(boxCentres[i]));
	    }
	}
	

	TEST_F(PCLUtilTest, indexunflatten) {
	    int x, y, z;
	    cube3.indexUnflatten(0, x, y, z);
	    ASSERT_EQ(0, x);
	    ASSERT_EQ(0, y);
	    ASSERT_EQ(0, z);

	    cube3.indexUnflatten(2, x, y, z);
	    ASSERT_EQ(2, x);
	    ASSERT_EQ(0, y);
	    ASSERT_EQ(0, z);

	    cube3.indexUnflatten(10, x, y, z);
	    ASSERT_EQ(1, x);
	    ASSERT_EQ(0, y);
	    ASSERT_EQ(1, z);

	    cube3.indexUnflatten(18, x, y, z);
	    ASSERT_EQ(0, x);
	    ASSERT_EQ(0, y);
	    ASSERT_EQ(2, z);

	    cube3.indexUnflatten(26, x, y, z);
	    ASSERT_EQ(2, x);
	    ASSERT_EQ(2, y);
	    ASSERT_EQ(2, z);
	}
	
  } // namespace sysutil
} // namespace testing
