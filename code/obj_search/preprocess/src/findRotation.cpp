#include <simple_xml_parser.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <ros/console.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include "sysutil/sysutil.hpp"
#include "rosutil/rosutil.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "loadtest");
    ros::NodeHandle handle;
    
    std::string cloudDir;
    ROSUtil::getParam(handle, "/rotationtest/cloud_dir", cloudDir);
    float floorOffset;
    ROSUtil::getParam(handle, "/rotationtest/floor_offset", floorOffset);
    float ceilingOffset;
    ROSUtil::getParam(handle, "/rotationtest/ceiling_offset", ceilingOffset);
    float floorZ;
    ROSUtil::getParam(handle, "/obj_search/floor_z", floorZ);
    float ceilingZ;
    ROSUtil::getParam(handle, "/obj_search/ceiling_z", ceilingZ);

    std::string roomXML = std::string(SysUtil::fullDirPath(cloudDir)) + "room.xml";
    std::string roomCloud = std::string(SysUtil::fullDirPath(cloudDir)) + "complete_cloud.pcd";
        
    SimpleXMLParser<pcl::PointXYZRGB> parser;
    ROS_INFO("Starting load");
    SimpleXMLParser<pcl::PointXYZRGB>::RoomData roomData = parser.loadRoomFromXML(roomXML);
    ROS_INFO("Load complete.");
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(roomCloud, *originalCloud) != -1){
	std::cout << "Loaded cloud from " << roomCloud << std::endl;
    } else {
	std::cout << "Could not load cloud from " << roomCloud << std::endl;
	exit(1);
    }

    tf::StampedTransform roomRotation = roomData.vIntermediateRoomCloudTransforms[0];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud(*originalCloud, *transformedCloud, roomRotation);

    // pcl::PointXYZRGB min;
    // pcl::PointXYZRGB max;
    // pcl::getMinMax3D(*transformedCloud, min, max);

    // ROS_INFO("min: %f, %f, %f", min.x, min.y, min.z);
    // ROS_INFO("max: %f, %f, %f", max.x, max.y, max.z);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trimmedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(transformedCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(floorZ + floorOffset, ceilingZ - ceilingOffset);
    pass.filter(*trimmedCloud);

    pcl::PCDWriter writer;
    ROS_INFO("Writing transformed cloud...");
    writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(cloudDir) + "transformedRoom.pcd", *transformedCloud, true);
    ROS_INFO("Done");
    ROS_INFO("Writing trimmed cloud...");
    writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(cloudDir) + "trimmedRoom.pcd", *trimmedCloud, true);
    ROS_INFO("Done");
    
    return 0;
}
