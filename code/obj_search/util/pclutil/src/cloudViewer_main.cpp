#include "pclutil/cloudViewer.hpp"

// blue, yellow, magenta, orange
//static const std::vector<int> colours({0x0279ff, 0xf2ff00, 0xff0074, 0xff9a00});
// dark blue, light blue, purple, dark yellow
//static const std::vector<int> colours({0x3403ff, 0x0271ff, 0xae01ff, 0xffcf00});
// pastel blue, green, purple, yellow
static const std::vector<int> colours({0x6275df, 0x52e57e, 0xc852dc, 0xffd05b});

/** 
 * View a cloud with its normals
 * 
 * @param cloudFile The file of the cloud
 * @param normalFile file of normals
 * @param cameraFile camera file to use, if any
 */
void viewNormals(std::string cloudFile, std::string normalFile, std::string cameraFile="NULL") {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PCDReader reader;
    reader.read(cloudFile, *cloud);
    reader.read(normalFile, *normals);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Cloud Viewer"));
    viewer->setBackgroundColor(1, 1, 1);


    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    if (cameraFile.compare("NULL") != 0){
	viewer->loadCameraParameters(cameraFile);
    } else {
	viewer->initCameraParameters();
    }
    
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");

    int r, g, b;
    objsearch::pclutil::hexToRGB(colours[3], r, g, b);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r/255.0, g/255.0, b/255.0, "normals"); 
	    

	    
    while (!viewer->wasStopped()) {
    	viewer->spinOnce(100);
    	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

/** 
 * View multiple clouds in the same viewer.
 * 
 * @param cloudFiles vector of the files to view - can also contain a camera setting
 * @param cameraFile if true, use the last file in the cloud array as a camera setting
 * @param colour if true, apply preset colours to the clouds
 * @param showOrigin if true, show the origin
 */
void viewClouds(std::vector<std::string> cloudFiles, bool cameraFile, bool colour, bool showOrigin) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PCDReader reader;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Cloud Viewer"));
    for (size_t i = 0; i < cloudFiles.size(); i++) {
	if (i + 1 == cloudFiles.size() && cameraFile) {
	    break;
	}
	reader.read(cloudFiles[i], *cloud);
	if (colour && i >=colours.size()) { // if more clouds than defined colours, go random
	    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> rgb(cloud);
	    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, std::string("cloud" + std::to_string(i)));
	} else if (colour) { // defined colour for this cloud index
	    int r, g, b;
	    objsearch::pclutil::hexToRGB(colours[i], r, g, b);
	    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(cloud, r, g, b);
	    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, std::string("cloud" + std::to_string(i)));
	} else {
	    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, std::string("cloud" + std::to_string(i)));
	}


    }

    viewer->setBackgroundColor(1.0, 1.0, 1.0);
    if (showOrigin) {
	viewer->addCoordinateSystem(3.0);
    }
    if (cameraFile) {
	viewer->loadCameraParameters(cloudFiles.back());
    } else {
	viewer->initCameraParameters();
    }
    
    viewer->setShowFPS(false);

    while (!viewer->wasStopped()) {
    	viewer->spinOnce(100);
    	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

/** 
 * View a single cloud from the given camera angle
 * 
 * @param cloudFile cloud to view
 * @param cameraFile camera position from which to view cloud
 */
void viewCloudFromCamera(std::string cloudFile, std::string cameraFile) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PCDReader reader;
    reader.read(cloudFile, *cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Cloud Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
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

    if (req[1] == 'n') {
	if (req.find('c') != std::string::npos){
	    viewNormals(argv[2], argv[3], argv[4]);
	} else {
	    viewNormals(argv[2], argv[3]);
	}
    } else if (req[1] == 'm') {
	std::vector<std::string> clouds;
	for (int i = 2; i < argc; i++) {
	    clouds.push_back(argv[i]);
	}

	bool camera = req.find('c') != std::string::npos ? true : false;
	bool colour = req.find('l') != std::string::npos ? true : false;
	bool origin = req.find('o') != std::string::npos ? true : false;
	viewClouds(clouds, camera, colour, origin);
    }
    
    return 0;
}

