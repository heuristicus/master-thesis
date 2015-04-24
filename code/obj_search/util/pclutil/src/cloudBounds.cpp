#include "pclutil/cloudBounds.hpp"

namespace objsearch {
    namespace pclutil {

	int OrientedBoundingBox::id = 0;

	OrientedBoundingBox::OrientedBoundingBox(
	    pcl::PointXYZ _position, pcl::PointXYZ _extents, Eigen::Matrix3f _rotation,
	    Eigen::Vector3f _major, Eigen::Vector3f _middle, Eigen::Vector3f _minor,
	    std::string _label)
	    : position(_position), extents(_extents), rotation(_rotation),
	      v_major(_major), v_middle(_middle), v_minor(_minor) {
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

	    // if the 
	    if (_label.compare("nil") == 0) {
		label = std::to_string(id);
	    } else {
		label = _label;
	    }

	    id++; // increment the ID for the next box
	}

	/** 
	 * 
	 * 
	 * @param viewer 
	 */
	void OrientedBoundingBox::addToViewer(pcl::visualization::PCLVisualizer::Ptr& viewer){
	    Eigen::Vector3f pos(position.x, position.y, position.z);
	    Eigen::Quaternionf quat(rotation);
	    viewer->addCube(pos, quat, extents.x * 2, extents.y * 2,
			    extents.z * 2, std::string(label + "_OBB").c_str());

	    // define points at the ends of the eigenvectors to show the coordinate system
	    pcl::PointXYZ x_axis(v_major(0) + pos(0), v_major(1) + pos(1),
				 v_major(2) + pos(2));
	    pcl::PointXYZ y_axis(v_middle(0) + pos(0), v_middle(1) + pos(1),
				 v_middle(2) + pos(2));
	    pcl::PointXYZ z_axis(v_minor(0) + pos(0), v_minor(1) + pos(1),
				 v_minor(2) + pos(2));
	    viewer->addLine(position, x_axis, 1.0f, 0.0f, 0.0f, std::string(label + "_maj_eig").c_str());
	    viewer->addLine(position, y_axis, 0.0f, 1.0f, 0.0f, std::string(label + "_mid_eig").c_str());
	    viewer->addLine(position, z_axis, 0.0f, 0.0f, 1.0f, std::string(label + "_min_eig").c_str());
	}

	/** 
	 * 
	 * 
	 * @param cloud 
	 * 
	 * @return 
	 */
	OrientedBoundingBox getOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
						   std::string label){
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
	    pcl::PointXYZ extents((max_point_OBB.x - min_point_OBB.x)/2,
				  (max_point_OBB.y - min_point_OBB.y)/2,
				  (max_point_OBB.z - min_point_OBB.z)/2);
	    return OrientedBoundingBox(position_OBB, extents, rotational_matrix_OBB,
				       major_vector, middle_vector, minor_vector, label);
	}
    } // namespace pclutil
} // namespace objsearch
