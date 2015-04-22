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
    struct OrientedBoundingBox {
	OrientedBoundingBox(const pcl::PointXYZ _position, const pcl::PointXYZ _extents,
			    const Eigen::Matrix3f _rotation, const Eigen::Vector3f _major,
			    const Eigen::Vector3f _middle, const Eigen::Vector3f _minor)
	    : position(_position), extents(_extents), rotation(_rotation), v_major(_major),
	      v_middle(_middle), v_minor(_minor) {}

	pcl::PointXYZ position;
	pcl::PointXYZ extents; // half the width, height and depth of the box
	Eigen::Matrix3f rotation;
	Eigen::Vector3f v_major, v_middle, v_minor;
    };

    // assumed that the point is already in the box frame.
    template <typename PointT>
    bool pointInBbox(const PointT& point, const OrientedBoundingBox& bbox){
	return std::fabs(point.x) < bbox.extents.x
				    && std::fabs(point.y) < bbox.extents.y
				    && std::fabs(point.z) < bbox.extents.z;
    }

    OrientedBoundingBox getOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;

	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);

	std::cout << position_OBB << std::endl;
	std::cout << rotational_matrix_OBB << std::endl;

	pcl::PointXYZ extents((max_point_OBB.x - min_point_OBB.x)/2, (max_point_OBB.y - min_point_OBB.y)/2, (max_point_OBB.z - min_point_OBB.z)/2);
	return OrientedBoundingBox(position_OBB, extents, rotational_matrix_OBB, major_vector, middle_vector, minor_vector);
    }

    void viewCloudWithBBOX(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const OrientedBoundingBox& bbox){

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

  Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
	  t(i,j) = bbox.rotation(i,j);
      }
  }
  t(0,3) = bbox.position.x;
  t(1,3) = bbox.position.y;
  t(2,3) = bbox.position.z;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr testcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedcloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  if (argc > 2) {
      if(pcl::io::loadPCDFile(argv[2], *testcloud) == -1)
	  return(-1);

      // rotate all points in the test cloud according to the inverse transform
      // matrix of the oriented bounding box. This puts all points into the
      // coordinate system of the reference frame of the box
      Eigen::Matrix4f ti = t.inverse();
      pcl::transformPointCloud(*testcloud, *transformedcloud, ti);
      std::vector<int> indices;
      for (int i = 0; i < transformedcloud->size(); i++) {
	  if (pclutil::pointInBbox<pcl::PointXYZRGB>(transformedcloud->points[i], bbox)) {
	      indices.push_back(i);
	  }
      }

      for (int i = 0; i < indices.size(); i++) {
	  pcl::PointXYZRGB& p = testcloud->points[indices[i]];
	  p.r = 255;
	  p.g = 0;
	  p.b = 0;
      }

      pcl::PCDWriter writer;
      writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(SysUtil::trimPath(std::string(argv[1]), 1)) +
				     "boxcoloured.pcd",
				     *(testcloud), true);
  }

  std::cout << t << std::endl;
  std::cout << bbox.rotation << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(transformedcloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(testcloud);
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
//  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  if (argc > 2) {
      viewer->addPointCloud<pcl::PointXYZRGB>(testcloud, rgb2, "test");
  }
  
//  viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

  Eigen::Vector3f position(bbox.position.x, bbox.position.y, bbox.position.z);
  Eigen::Quaternionf quat(bbox.rotation);
  viewer->addCube(position, quat, bbox.extents.x * 2, bbox.extents.y * 2, bbox.extents.z * 2, "OBB");

  std::cout << bbox.position << std::endl;
  std::cout << bbox.rotation << std::endl;

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
