#include "pclutil/cloudViewer.hpp"

int main(int argc, char *argv[]) {
    bool haveNormals = argc == 3 ? true : false;

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PCDReader reader;
    reader.read(argv[1], *cloud);
    reader.read(argv[2], *normals);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    
    if (haveNormals) {
    	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
    }
	    
    viewer->initCameraParameters();
	    
    while (!viewer->wasStopped())
    {
    	viewer->spinOnce(100);
    	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    
    return 0;
}

