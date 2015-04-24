#include "pclutil/cloudBounds.hpp"

int main(int argc, char* argv[]) {
    using namespace objsearch::pclutil;
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
	    for (int i = 2; i < argc; i++) {
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
	
	OrientedBoundingBox bbox = getOrientedBoundingBox(annotation);

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
