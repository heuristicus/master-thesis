#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char *argv[]) {

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origCloud(
	new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trimmed(
	new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 1; i < argc; i++) {
	reader.read(argv[i], *origCloud);
	pass.setInputCloud(origCloud);
	pass.setFilterFieldName("x");
	// a little bit above the floor, and a little below the ceiling are our limits
	pass.setFilterLimits(-10, -1);
	// filter the values outside the thresholds and put the rest of the
	// points back into the cloud.
	pass.filter(*trimmed);
	writer.write<pcl::PointXYZRGB>(std::string(argv[i]), *trimmed, true);
    }
}


