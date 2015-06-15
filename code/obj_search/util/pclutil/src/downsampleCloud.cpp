#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


template <typename Desc>
std::vector<int> getHistogram(const std::vector<std::string> descFiles) {
}


int main(int argc, char *argv[]) {
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    std::string fname(argv[2]);
    std::string leaf(argv[1]);
    float leaff = std::stof(leaf);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dscloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    reader.read(fname, *cloud);
    
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaff, leaff, leaff);
    sor.filter(*dscloud);

    writer.write<pcl::PointXYZRGB>(std::string(fname, 0, fname.size()-4) + "dsample.pcd", *dscloud, true);
}
