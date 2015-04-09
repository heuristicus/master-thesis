#include <iostream>

#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>

#include <load_utilities.h>

#include "sysutil/sysutil.hpp"

namespace smlu = semantic_map_load_utilties;

int main(int argc, char *argv[]) {
    smlu::IntermediateCloudCompleteData<pcl::PointXYZRGB> inter = smlu::loadIntermediateCloudsCompleteDataFromSingleSweep<pcl::PointXYZRGB>(std::string(argv[1]), true);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intermediateCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (int i = 0; i < inter.vIntermediateRoomClouds.size(); i++) {
	std::cout << "cloud " << i << " of " << inter.vIntermediateRoomClouds.size() << std::endl;
	pcl_ros::transformPointCloud(*(inter.vIntermediateRoomClouds[i]), *intermediateCloud, inter.vIntermediateRoomCloudTransforms[i]);
	*mergedCloud += *intermediateCloud;

	intermediateCloud->clear();
    }

    pcl::PCDWriter writer;
    std::string out = SysUtil::fullDirPath(SysUtil::trimPath(std::string(argv[1]), 1)) + "mergedCloud.pcd";
    std::cout << "Writing to " << out.c_str() << std::endl;
    writer.write<pcl::PointXYZRGB>(out, *mergedCloud, true);
    
    return 0;
}
