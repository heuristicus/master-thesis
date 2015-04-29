#include "pclutil/grid3d.hpp"
#include "pclutil/annotationExtract.hpp"
#include "pclutil/cloudBounds.hpp"
#include "pclutil/cloudViewer.hpp"
#include "pclutil/pointValidation.hpp"
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <gtest/gtest.h>

namespace testing {
    namespace pclutil {
	using namespace objsearch::pclutil;
	
	TEST(pclutilTest, grid3d_init_cube){
	    Grid3D grid(1.5, 0.5);
	    ASSERT_TRUE(grid.width_ == 3);
	    ASSERT_TRUE(grid.height_ == 3);
	    ASSERT_TRUE(grid.depth_ == 3);
	    ASSERT_TRUE(grid.values_.size() == 27);
	}

	TEST(pclutilTest, grid3d_init_box){
	    Grid3D grid2(3, 2, 1, 1, 1, 1);
	    ASSERT_TRUE(grid2.width_ == 3);
	    ASSERT_TRUE(grid2.height_ == 1);
	    ASSERT_TRUE(grid2.depth_ == 2);
	    ASSERT_TRUE(grid2.values_.size() == 6);
	}

	// make sure the initialisation works as expected when the dimension is
	// not exactly divisible by the step.
	TEST(pclutilTest, grid3d_init_box_overstep){
	    Grid3D grid(2, 2, 2, 0.6, 0.3, 0.7);
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
	TEST(pclutilTest, grid3d_cellcentre_corners){
	    Grid3D grid(3, 3, 3, 1, 1, 1);
	    pcl::PointXYZ frontBottomLeft(0.5, 0.5, 0.5);
	    pcl::PointXYZ frontBottomRight(2.5, 0.5, 0.5);
	    pcl::PointXYZ frontTopLeft(0.5, 0.5, 2.5);
	    pcl::PointXYZ frontTopRight(2.5, 0.5, 2.5);

	    pcl::PointXYZ backBottomLeft(0.5, 2.5, 0.5);
	    pcl::PointXYZ backBottomRight(2.5, 2.5, 0.5);
	    pcl::PointXYZ backTopLeft(0.5, 2.5, 2.5);
	    pcl::PointXYZ backTopRight(2.5, 2.5, 2.5);

	    pointEqual(frontBottomLeft, grid.cellCentre(0.1, 0.1, 0.1));
	    pointEqual(frontBottomRight, grid.cellCentre(2.1, 0.1, 0.1));
	    pointEqual(frontTopLeft, grid.cellCentre(0.1, 0.1, 2.1));
	    pointEqual(frontTopRight, grid.cellCentre(2.1, 0.1, 2.1));
	  
	    pointEqual(backBottomLeft, grid.cellCentre(0.1, 2.1, 0.1));
	    pointEqual(backBottomRight, grid.cellCentre(2.1, 2.1, 0.1));
	    pointEqual(backTopLeft, grid.cellCentre(0.1, 2.1, 2.1));
	    pointEqual(backTopRight, grid.cellCentre(2.1, 2.1, 2.1));
	}

	// Make sure the cell centre computation works for all edges of a simple cube
	TEST(pclutilTest, grid3d_cellcentre_edges){
	    Grid3D grid(3, 3, 3, 1, 1, 1);
	    pcl::PointXYZ frontBottom(1.5, 0.5, 0.5);
	    pcl::PointXYZ frontTop(1.5, 0.5, 2.5);
	    pcl::PointXYZ frontLeft(0.5, 0.5, 1.5);
	    pcl::PointXYZ frontRight(2.5, 0.5, 1.5);

	    pcl::PointXYZ backBottom(1.5, 2.5, 0.5);
	    pcl::PointXYZ backTop(1.5, 2.5, 2.5);
	    pcl::PointXYZ backLeft(0.5, 2.5, 1.5);
	    pcl::PointXYZ backRight(2.5, 2.5, 1.5);

	    pointEqual(frontBottom, grid.cellCentre(1.1, 0.1, 0.1));
	    pointEqual(frontTop, grid.cellCentre(1.1, 0.1, 2.1));
	    pointEqual(frontLeft, grid.cellCentre(0.1, 0.1, 1.1));
	    pointEqual(frontRight, grid.cellCentre(2.1, 0.1, 1.1));
	  
	    pointEqual(backBottom, grid.cellCentre(1.1, 2.1, 0.1));
	    pointEqual(backTop, grid.cellCentre(1.1, 2.1, 2.1));
	    pointEqual(backLeft, grid.cellCentre(0.1, 2.1, 1.1));
	    pointEqual(backRight, grid.cellCentre(2.1, 2.1, 1.1));
	}
	
	// Make sure the cell centre computation works for the centre of a simple cube
	TEST(pclutilTest, grid3d_cellcentre_centre){
	    Grid3D grid(3, 3, 3, 1, 1, 1);
	    pcl::PointXYZ centre(1.5, 1.5, 1.5);
	    pointEqual(centre, grid.cellCentre(1.1, 1.1, 1.1));
	}
  } // namespace sysutil
} // namespace testing
