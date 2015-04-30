#include <iostream>
#include <string>
#include <algorithm>

#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>

#include <load_utilities.h>

#include "sysutil/sysutil.hpp"

namespace smlu = semantic_map_load_utilties;
namespace sys = objsearch::sysutil;

int main(int argc, char *argv[]) {
    // first argument should be the intermediate cloud to rotate. The rotation
    // will be extracted from the xml in the corresponding directory
    std::string interFile = std::string(argv[1]);
    int startInd = interFile.find_last_of('.') - 4;// start index of the number in the filename
    // the number of the intermediate cloud
    std::string number = std::string(interFile.begin() + startInd,
				     interFile.begin() + startInd + 4);

    std::string dir = sys::fullDirPath(sys::trimPath(interFile, 1));
    std::string xml = dir + "room.xml";

    SimpleXMLParser<pcl::PointXYZRGB> parser;
    SimpleXMLParser<pcl::PointXYZRGB>::RoomData rd = parser.loadRoomFromXML(xml);

    std::vector<std::string> intermediateFiles = sys::listFilesWithString(dir, "intermediate");
    std::sort(intermediateFiles.begin(), intermediateFiles.end());
    // find the index in the vector of the file which we are interested in so
    // that its rotation can be extracted.
    int ind;
    for (size_t i = 0; i < intermediateFiles.size(); i++) {
	if (intermediateFiles[i].find(number) != std::string::npos) {
	    ind = i;
	    break;
	}
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intermediateCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // transform the intermediate cloud into the same frame as the registered
    // (complete) cloud
    pcl_ros::transformPointCloud(*(rd.vIntermediateRoomClouds[ind]),
    				 *intermediateCloud,
				 rd.vIntermediateRoomCloudTransformsRegistered[ind]);
    // transform from that frame using the same transform used on the complete
    // cloud to generate the trimmed cloud.
    pcl_ros::transformPointCloud(*intermediateCloud,
    				 *finalCloud,
				 rd.vIntermediateRoomCloudTransforms[0]);


    pcl::PCDWriter writer;
    std::string out = dir + sys::removeExtension(sys::trimPath(interFile, -1))
	+ "_rotated.pcd";
    std::cout << "Writing to " << out.c_str() << std::endl;
    writer.write<pcl::PointXYZRGB>(out, *finalCloud, true);
    
    return 0;
}
