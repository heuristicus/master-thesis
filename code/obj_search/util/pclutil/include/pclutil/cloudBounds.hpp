#ifndef CLOUD_BOUNDS_H
#define CLOUD_BOUNDS_H

#include <boost/thread/thread.hpp>

#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <sysutil/sysutil.hpp>

namespace objsearch {
    namespace pclutil {
	class OrientedBoundingBox {
	private:
	    static int id;
	public:
	    OrientedBoundingBox(){};
	    OrientedBoundingBox(pcl::PointXYZ _position, pcl::PointXYZ _extents,
				Eigen::Matrix3f _rotation, Eigen::Vector3f _major,
				Eigen::Vector3f _middle, Eigen::Vector3f _minor,
				std::string _label="nil");
	    const pcl::PointXYZ position; // position of the centre of the box
	    const pcl::PointXYZ extents; // half the width, height and depth of the box
	    const Eigen::Matrix3f rotation; // the rotation of the box relative to the origin(?)
	    const Eigen::Vector3f v_major, v_middle, v_minor; // the three eigenvectors
	    std::string label; // mostly used to ensure uniqueness when adding to visualisation
	    Eigen::Matrix4f transform; // transform combining the position and rotation
	    Eigen::Matrix4f transformInverse; // inverse of the above

	    /** 
	     * Check whether the given point is inside the oriented bounding box.
	     * 
	     * @param point The point to check
	     * @param inBoxFrame Set to true if the point has already been
	     * transformed into the OBB frame of reference. Default is false.
	     * 
	     * @return True if the points is within the bounds of the OBB.
	     */
	    template <typename PointT>
	    bool contains(PointT point, bool inBoxFrame) {
		if (!inBoxFrame) {
		    point = toBoxFrame(point);
		}

		// once the point is in the box frame, all that needs to be done is
		// to check that the point is within the extents for all axes.
		return (std::fabs(point.x) < extents.x
			&& std::fabs(point.y) < extents.y
			&& std::fabs(point.z) < extents.z);
	    }
    
	    /** 
	     * Transform a point from its current reference frame to the frame of
	     * the OBB
	     * 
	     * @param point The point to transform
	     * 
	     * @return The point with its values modified so that the origin now
	     * corresponds to the OBB frame defined by the eigenvectors
	     */
	    template <typename PointT>
	    PointT toBoxFrame(PointT point) {
		// transform the point to homogeneous coordinates
		Eigen::Vector4f pvec(point.x, point.y, point.z, 1);
		// transforming points into the box space, so use inverse
		Eigen::Vector4f r = transformInverse*pvec;
		// modify the coordinates of the point so that other values the
		// point might have are preserved
		point.x = r(0);
		point.y = r(1);
		point.z = r(2);
		return point;
	    }

	
	    void addToViewer(pcl::visualization::PCLVisualizer::Ptr& viewer);
	};

	OrientedBoundingBox getOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
						   std::string label="nil");
    } // namespace pclutil
} // namespace objsearch

#endif // CLOUD_BOUNDS_H




