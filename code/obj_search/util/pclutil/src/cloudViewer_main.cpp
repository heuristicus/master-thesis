#include "pclutil/cloudViewer.hpp"


void viewNormals(std::string cloudFile, std::string normalFile) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PCDReader reader;
    reader.read(cloudFile, *cloud);
    reader.read(normalFile, *normals);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	    
    viewer->initCameraParameters();
	    
    while (!viewer->wasStopped()) {
    	viewer->spinOnce(100);
    	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void viewCloudFromCamera(std::string cloudFile, std::string cameraFile) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PCDReader reader;
    reader.read(cloudFile, *cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Cloud Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
    viewer->loadCameraParameters(cameraFile);
    viewer->setShowFPS(false);

    while (!viewer->wasStopped()) {
    	viewer->spinOnce(100);
    	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char *argv[]) {

    std::string req(argv[1]);
    if (req[0] != '-') { // must provide option as first parameter
	std::cout << "You must provide an option as the first parameter." << std::endl;
	std::cout << objsearch::pclutil::viewerHelp << std::endl;
    }

    if (req.find('n') != std::string::npos) {
	viewNormals(argv[2], argv[3]);
    } else if (req.find('c') != std::string::npos) {
	viewCloudFromCamera(argv[2], argv[3]);
    }
        
    return 0;
}

