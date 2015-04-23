#include "pclutil/cloudBounds.hpp"

namespace pclutil {

    int OrientedBoundingBox::id = 0;

    OrientedBoundingBox::OrientedBoundingBox(
	pcl::PointXYZ _position,pcl::PointXYZ _extents,	Eigen::Matrix3f _rotation,
	Eigen::Vector3f _major,	Eigen::Vector3f _middle, Eigen::Vector3f _minor,
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
	pcl::PointXYZ extents((max_point_OBB.x - min_point_OBB.x)/2,
			      (max_point_OBB.y - min_point_OBB.y)/2,
			      (max_point_OBB.z - min_point_OBB.z)/2);
	return OrientedBoundingBox(position_OBB, extents, rotational_matrix_OBB,
				   major_vector, middle_vector, minor_vector);
    }

    float boxVolume(pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint) {
	return (maxPoint.x - minPoint.x) * (maxPoint.y - minPoint.y) * (maxPoint.z - minPoint.z);
    }

} // namespace pclutil

int main(int argc, char* argv[]) {
    // the first arg is the directory/file on which to operate, additional ones
    // are optional and specify which cloud/directory containing clouds for
    // which the bbox is to be found. If directories are given as arguments,
    // assumptions are made about what to use. The overlap check cloud will be
    // nonPlanes.pcd, and the annotations checked will contain the string "label"
    
    std::string operate(argv[1]);
    std::vector<std::string> bboxFiles;
    if (!SysUtil::isFile(operate)) {
	// only have directory, so get all the annotation cloud files there
	if (argc == 2){
	    bboxFiles = SysUtil::listFilesWithString(operate, std::regex(".*label.*pcd"));
	}
	
	// operate is a directory, so construct the path to the file
	operate = SysUtil::fullDirPath(operate) + "nonPlanes.pcd";
    }

    // have additional arguments
    if (argc > 2) {
	// have a directory as additional arguments - extract all annotation
	// cloud files there
	if (!SysUtil::isFile(argv[2])) {
	    bboxFiles = SysUtil::listFilesWithString(std::string(argv[2]), std::regex(".*label.*pcd"));
	} else { // additional arguments are assumed to be files
	    for (int i = 0; i < argc; i++) {
		bboxFiles.push_back(std::string(argv[i]));
	    }
	}
    }

    // this is the cloud which we want to check for overlap with the bounding boxes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PCDReader reader;
    if (reader.read(operate, *cloud) < 0){
	std::cout << "Could not read cloud from " << operate.c_str() << std::endl;
	exit(1);
    }

    // this is the cloud which will hold the annotations
    pcl::PointCloud<pcl::PointXYZ>::Ptr annotation(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
	viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // store points transformed to the OBB reference frame here
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZRGB>());
    // will colour points in this cloud inside bounding boxes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloured = cloud->makeShared();

    // use to randomly colour points for different boxes
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,255);
    for (size_t i = 0; i < bboxFiles.size(); i++) {
	if (reader.read(bboxFiles[i], *annotation) < 0){
	    std::cout << "Could not read annotation cloud from " << bboxFiles[i] << std::endl;
	    exit(1);
	}
	
	pclutil::OrientedBoundingBox bbox = pclutil::getOrientedBoundingBox(annotation);

	std::vector<int> indices;
	// rotate all points in the test cloud according to the inverse transform
	// matrix of the oriented bounding box. This puts all points into the
	// coordinate system of the reference frame of the box
	// using the pcl transformpointcloud with the bbox inverse transform
	pcl::transformPointCloud(*cloud, *transformed, bbox.transformInverse);
	
	for (size_t i = 0; i < transformed->size(); i++) {
	    if (bbox.contains(transformed->points[i], true)) {
		indices.push_back(i);
	    }
	}

	int r = distribution(generator);
	int g = distribution(generator);
	int b = distribution(generator);
	for (size_t i = 0; i < indices.size(); i++) {
	    pcl::PointXYZRGB& p = coloured->points[indices[i]];
	    p.r = r;
	    p.g = g;
	    p.b = b;
	}

	// add the bounding box for this cloud to the viewer
	bbox.addToViewer(viewer);
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(coloured);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->addPointCloud<pcl::PointXYZRGB>(coloured, rgb, "c");


    while(!viewer->wasStopped())
    {
	viewer->spinOnce(100);
	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return(0);
}
