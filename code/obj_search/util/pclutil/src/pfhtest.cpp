#include <random>
#include <iostream>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/common.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/usc.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

int main(int argc, char *argv[]) {
    pcl::console::TicToc timer;
    pcl::console::TicToc subTimer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smallCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDWriter writer;
    pcl::PCDReader reader;
    std::default_random_engine generator;
    std::uniform_int_distribution<int> rgb(0, 255);

    bool createCloud = false;
    
    if (argc == 1) {
	createCloud = true;
    }

    timer.tic();

    std::cout << "==========---------- Small cloud ----------==========" << std::endl;
    if (createCloud) {
	std::cout << "Generating cloud" << std::endl;
	smallCloud->width    = 30000;
	smallCloud->height   = 1;
	smallCloud->is_dense = false;
	smallCloud->points.resize(smallCloud->width * smallCloud->height);

	// make a cube
	std::uniform_real_distribution<float> xDistSmall(0, 1);
	std::uniform_real_distribution<float> yDistSmall(0, 1);
	std::uniform_real_distribution<float> zDistSmall(0, 1);
	int xOff = 2, zOff = 1, yOff = 2.5; // base for the cube

	subTimer.tic();
	// generate points uniformly in the space 
	for (size_t i = 0; i < smallCloud->points.size(); ++i) {
	    smallCloud->points[i].x = xOff + xDistSmall(generator);
	    smallCloud->points[i].y = yOff + yDistSmall(generator);
	    smallCloud->points[i].z = zOff + zDistSmall(generator);
	    smallCloud->points[i].r = rgb(generator);
	    smallCloud->points[i].g = rgb(generator);
	    smallCloud->points[i].b = rgb(generator);
	}
	std::cout << "Small cloud generation took " << subTimer.toc() << std::endl;
	subTimer.tic();
	writer.write<pcl::PointXYZRGB>("smallCloud.pcd", *smallCloud, true);
	subTimer.toc();
    } else {
	std::cout << "Reading cloud from file" << std::endl;
	subTimer.tic();
	reader.read(argv[1], *smallCloud);
	std::cout << "Small cloud reading took " << subTimer.toc() << std::endl;
    }

    std::cout << "Estimating normals" << std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::Normal>::Ptr smallNormals(new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud(smallCloud);
    ne.setSearchMethod(kdtree);
    ne.setRadiusSearch(0.08);
    subTimer.tic();
    ne.compute(*smallNormals);
    std::cout << "small cloud normal computation took " << subTimer.toc() << std::endl;

    subTimer.tic();
    writer.write<pcl::Normal>("smallCloud_normals.pcd", *smallNormals);
    std::cout << "Writing normals cloud took " << subTimer.toc() << std::endl;
    
    // downsample the clouds to get points at which features are to be computed
    std::cout << "Downsampling" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr descLocSmall(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> vgrid;
    float leafSize = 0.05;
    vgrid.setInputCloud(smallCloud);
    vgrid.setLeafSize(leafSize, leafSize, leafSize);
    subTimer.tic();
    vgrid.filter(*descLocSmall);
    std::cout << "Small cloud downsample took " << subTimer.toc() << std::endl;
    std::cout << "Features for small cloud will be computed at " << (int)descLocSmall->size() << " points" << std::endl;

    subTimer.tic();
    writer.write<pcl::PointXYZRGB>("smallCloud_descLocs.pcd", *descLocSmall, true);
    std::cout << "Writing feature location cloud took " << subTimer.toc() << std::endl;

    std::cout << "Computing features" << std::endl;
    pcl::PointCloud<pcl::PFHSignature125>::Ptr smallDesc(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
		    
    pfh.setInputCloud(descLocSmall); // compute features at these locations
    pfh.setSearchSurface(smallCloud); // using this cloud to define the local regions of those points
    pfh.setInputNormals(smallNormals);
    pfh.setSearchMethod(kdtree);
    pfh.setRadiusSearch(0.05);

    subTimer.tic();
    pfh.compute(*smallDesc);
    std::cout << "Small cloud feature computation took " << subTimer.toc() << std::endl;

    subTimer.tic();
    writer.write<pcl::PFHSignature125>("smallCloud_features.pcd", *smallDesc, true);
    std::cout << "Writing feature cloud took " << subTimer.toc() << std::endl;

    std::cout << "Total time for small cloud: " << timer.toc() << std::endl;

    /**
     * Large cloud
     */
    std::cout << "==========---------- Large cloud ----------==========" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr largeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (createCloud) {
	std::cout << "Generating large cloud" << std::endl;
	timer.tic();
	largeCloud->width    = 400000;
	largeCloud->height   = 1;
	largeCloud->is_dense = false;
	largeCloud->points.resize(largeCloud->width * largeCloud->height);
	std::uniform_real_distribution<float> xDistLarge(0, 4);
	std::uniform_real_distribution<float> yDistLarge(0, 6);
	std::uniform_real_distribution<float> zDistLarge(0, 3);

	subTimer.tic();
	// generate points uniformly in the space 
	for (size_t i = 0; i < largeCloud->points.size(); ++i) {
	    largeCloud->points[i].x = xDistLarge(generator);
	    largeCloud->points[i].y = yDistLarge(generator);
	    largeCloud->points[i].z = zDistLarge(generator);
	    largeCloud->points[i].r = rgb(generator);
	    largeCloud->points[i].g = rgb(generator);
	    largeCloud->points[i].b = rgb(generator);
	}
	std::cout << "Large cloud generation took " << subTimer.toc() << std::endl;
	subTimer.tic();
	writer.write<pcl::PointXYZRGB>("largeCloud.pcd", *largeCloud, true);
	std::cout << "Writing cloud took " << subTimer.toc() << std::endl;
    } else {
	std::cout << "Reading large cloud from file" << std::endl;
	subTimer.tic();
	reader.read(argv[2], *largeCloud);
	std::cout << "Large cloud reading took " << subTimer.toc() << std::endl;
    }

    std::cout << "Estimating normals" << std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr largeNormals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(largeCloud);
    ne.setSearchMethod(kdtree);
    ne.setRadiusSearch(0.08);
    subTimer.tic();
    ne.compute(*largeNormals);
    std::cout << "large cloud normal computation took " << subTimer.toc() << std::endl;

    subTimer.tic();
    writer.write<pcl::Normal>("largeCloud_normals.pcd", *largeNormals);
    std::cout << "Writing normals cloud took " << subTimer.toc() << std::endl;
    
    std::cout << "Downsampling" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr descLocLarge(new pcl::PointCloud<pcl::PointXYZRGB>());
    vgrid.setInputCloud(largeCloud);
    vgrid.setLeafSize(leafSize, leafSize, leafSize);
    subTimer.tic();
    vgrid.filter(*descLocLarge);
    std::cout << "Large cloud downsample took " << subTimer.toc() << std::endl;
    std::cout << "Features for large cloud will be computed at " << (int)descLocLarge->size() << " points" << std::endl;

    subTimer.tic();
    writer.write<pcl::PointXYZRGB>("largeCloud_descLocs.pcd", *descLocLarge, true);
    std::cout << "Writing features locations cloud took " << subTimer.toc() << std::endl;

    std::cout << "Computing features" << std::endl;
    pcl::PointCloud<pcl::PFHSignature125>::Ptr largeDesc(new pcl::PointCloud<pcl::PFHSignature125>());
    pfh.setInputCloud(descLocLarge); // compute features at these locations
    pfh.setSearchSurface(largeCloud); // using this cloud to define the local regions of those points
    pfh.setInputNormals(largeNormals);
    pfh.setSearchMethod(kdtree);
    pfh.setRadiusSearch(0.05);
    subTimer.tic();
    pfh.compute(*largeDesc);
    std::cout << "large cloud feature computation took " << subTimer.toc() << std::endl;

    subTimer.tic();
    writer.write<pcl::PFHSignature125>("largeCloud_features.pcd", *largeDesc, true);
    std::cout << "Writing feature cloud took " << subTimer.toc() << std::endl;
    
    std::cout << "Total time for large cloud: " << timer.toc() << std::endl;
} 
