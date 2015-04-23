#include <cmath>
#include <iostream>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/transforms.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

#include <sysutil/sysutil.hpp>

namespace pclutil {
    class OrientedBoundingBox {
    public:
	OrientedBoundingBox(const pcl::PointXYZ _position, const pcl::PointXYZ _extents,
			    const Eigen::Matrix3f _rotation, const Eigen::Vector3f _major,
			    const Eigen::Vector3f _middle, const Eigen::Vector3f _minor)
	    : position(_position), extents(_extents), rotation(_rotation), v_major(_major),
	      v_middle(_middle), v_minor(_minor) {
	    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	    for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
		    transform(i,j) = rotation(i,j);
		}
	    }
	    transform(0,3) = position.x;
	    transform(1,3) = position.y;
	    transform(2,3) = position.z;

	    transformInverse = transform.inverse();
	}
	

	const pcl::PointXYZ position;
	const pcl::PointXYZ extents; // half the width, height and depth of the box
	const Eigen::Matrix3f rotation;
	const Eigen::Vector3f v_major, v_middle, v_minor;
	Eigen::Matrix4f transform;
	Eigen::Matrix4f transformInverse;

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
	bool contains(PointT point, bool inBoxFrame=false) {
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
    };

    OrientedBoundingBox getOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;

	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);

	// extents are half the length of each side of the box
	pcl::PointXYZ extents((max_point_OBB.x - min_point_OBB.x)/2, (max_point_OBB.y - min_point_OBB.y)/2, (max_point_OBB.z - min_point_OBB.z)/2);
	return OrientedBoundingBox(position_OBB, extents, rotational_matrix_OBB, major_vector, middle_vector, minor_vector);
    }

    float boxVolume(pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint){
	return (maxPoint.x - minPoint.x) * (maxPoint.y - minPoint.y) * (maxPoint.z - minPoint.z);
    }

} // namespace pclutil

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if(pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	return(-1);

    pclutil::OrientedBoundingBox bbox = pclutil::getOrientedBoundingBox(cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr testcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedcloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedcloud2 (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colouredcloud1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colouredcloud2;

    std::cout << "made shared" << std::endl;

    if (argc > 2) {
	if(pcl::io::loadPCDFile(argv[2], *testcloud) == -1)
	    return(-1);

	colouredcloud1 = testcloud->makeShared();
	colouredcloud2 = testcloud->makeShared();

	pcl::PCDWriter writer;
	std::vector<int> indices;
	std::cout << "test cloud size" << (int)testcloud->size() << std::endl;
	std::cout << "transformed cloud 1 size" << (int)transformedcloud1->size() << std::endl;
	std::cout << "coloured cloud 1 size" << (int)colouredcloud1->size() << std::endl;
	// rotate all points in the test cloud according to the inverse transform
	// matrix of the oriented bounding box. This puts all points into the
	// coordinate system of the reference frame of the box
	// using the pcl transformpointcloud with the bbox inverse transform
	pcl::transformPointCloud(*testcloud, *transformedcloud1, bbox.transformInverse);

	
	for (int i = 0; i < transformedcloud1->size(); i++) {
	    if (bbox.contains(transformedcloud1->points[i], true)) {
		indices.push_back(i);
	    }
	}

	std::cout << "indices 1 size" << (int)indices.size() << std::endl;
      
	for (int i = 0; i < indices.size(); i++) {
	    pcl::PointXYZRGB& p = colouredcloud1->points[indices[i]];
	    p.r = 255;
	    p.g = 0;
	    p.b = 0;
	}

	std::cout << "transforming points" << std::endl;
	// using the bbox function
	transformedcloud2->resize(testcloud->size());
	for (int i = 0; i < testcloud->size(); i++) {
	    transformedcloud2->points[i] = bbox.toBoxFrame(testcloud->points[i]);
	}

	std::cout << "transformed cloud 2 size" << (int)transformedcloud2->size() << std::endl;
	std::cout << "coloured cloud 2 size" << (int)colouredcloud2->size() << std::endl;
	std::cout << "checking contains" << std::endl;
	indices.clear();
	for (int i = 0; i < transformedcloud2->size(); i++) {
	    if (i%10000 == 0) {
		std::cout << "i: " << i << std::endl;
		std::cout << "1: " << transformedcloud1->points[i] << std::endl;
		std::cout << "2: " << transformedcloud2->points[i] << std::endl;
	    }
	    if (bbox.contains(transformedcloud2->points[i], true)) {
		indices.push_back(i);
	    }
	}

	std::cout << "indices 2 size" << (int)indices.size() << std::endl;
	
	std::cout << "colourng" << std::endl;
	for (int i = 0; i < indices.size(); i++) {
	    pcl::PointXYZRGB& p = colouredcloud2->points[indices[i]];
	    p.r = 255;
	    p.g = 0;
	    p.b = 0;
	}
	std::cout << "writing" << std::endl;
      
	writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(SysUtil::trimPath(std::string(argv[1]), 1)) +
				       "boxcoloured1.pcd",
				       *(colouredcloud1), true);
	writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(SysUtil::trimPath(std::string(argv[1]), 1)) +
				       "boxcoloured2.pcd",
				       *(colouredcloud2), true);
	    
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(transformedcloud1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(transformedcloud2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb3(colouredcloud1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb4(colouredcloud2);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
//  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    if (argc > 2) {
	viewer->addPointCloud<pcl::PointXYZRGB>(transformedcloud1, rgb1, "tr1");
	viewer->addPointCloud<pcl::PointXYZRGB>(transformedcloud2, rgb2, "tr2");
	viewer->addPointCloud<pcl::PointXYZRGB>(colouredcloud1, rgb3, "c1");
	viewer->addPointCloud<pcl::PointXYZRGB>(colouredcloud2, rgb4, "c2");
    }
  
//  viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

    Eigen::Vector3f position(bbox.position.x, bbox.position.y, bbox.position.z);
    Eigen::Quaternionf quat(bbox.rotation);
    viewer->addCube(position, quat, bbox.extents.x * 2, bbox.extents.y * 2, bbox.extents.z * 2, "OBB");

    // eigenvector axis
    pcl::PointXYZ x_axis(bbox.v_major(0) + bbox.position.x, bbox.v_major(1) + bbox.position.y, bbox.v_major(2) + bbox.position.z);
    pcl::PointXYZ y_axis(bbox.v_middle(0) + bbox.position.x, bbox.v_middle(1) + bbox.position.y, bbox.v_middle(2) + bbox.position.z);
    pcl::PointXYZ z_axis(bbox.v_minor(0) + bbox.position.x, bbox.v_minor(1) + bbox.position.y, bbox.v_minor(2) + bbox.position.z);
    viewer->addLine(bbox.position, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine(bbox.position, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine(bbox.position, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    while(!viewer->wasStopped())
    {
	viewer->spinOnce(100);
	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return(0);
}
