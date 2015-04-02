/**
 * @file   featureExtraction.cpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Thu Mar 19 14:12:23 2015
 * 
 * @brief  
 * 
 * 
 */

#include "featureExtraction.hpp"

namespace objsearch {
    namespace featureExtraction {
	FeatureExtractor::FeatureExtractor(int argc, char *argv[]) {
	    ros::init(argc, argv, "feature_extraction");
	    ros::NodeHandle handle;

	    // Retrieve the directory containing the cloud to be processed
	    ROSUtil::getParam(handle, "/feature_extraction/input_cloud", cloudFile);
	    ROSUtil::getParam(handle, "/obj_search/processed_data_dir", processedDir);
	    ROSUtil::getParam(handle, "/obj_search/raw_data_dir", rawDir);
	    // If the given cloud file corresponds to a file in the processed data directory,
	    // extract the remaining directories in the path of the file so that the
	    // data can be put into the output directory with the same path.
	    if (cloudFile.compare(0, processedDir.size(), processedDir) == 0){
		dataSubDir = SysUtil::trimPath(std::string(cloudFile, processedDir.size()), 1);
	    } else if (cloudFile.compare(0, rawDir.size(), rawDir) == 0) {
		dataSubDir = SysUtil::trimPath(std::string(cloudFile, rawDir.size()), 1);
	    }

	    ROSUtil::getParam(handle, "/feature_extraction/output_dir", outDir);
	    // If output is not specified, set the output directory to be the processed
	    // data directory specified by the global parameters.
	    if (std::string("NULL").compare(outDir) == 0) {
		outDir = processedDir;
	    }

	    // The output path for processed clouds is the subdirectory combined
	    // with the top level output directory. If dataSubDir is not
	    // initialised, then clouds are simply output to the top level
	    // output directory
	    outPath = SysUtil::combinePaths(outDir, dataSubDir);

	    ROSUtil::getParam(handle, "/feature_extraction/feature_type", featureType);
	    extractFeatures();
	}

	void FeatureExtractor::extractFeatures(){
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloudFile, *cloud) != -1){
		std::cout << "Loaded cloud from " << cloudFile.c_str() << std::endl;
	    } else {
		std::cout << "Could not load cloud from " << cloudFile.c_str() << std::endl;
		exit(1);
	    }

	    // get lowercase for the feature type to allow lowercase input as well as
	    // uppercase, because laziness
	    std::transform(featureType.begin(), featureType.end(), featureType.begin(), ::tolower);
    
	    if (featureType.compare("shot") == 0) {
	
	    } else if (featureType.compare("usc") == 0) {
		// The shape context uses xyz points, so need to convert the cloud into that format
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    
		cloud_xyz->points.resize(cloud->size());
		int added = 0;

		ROS_INFO("Cloud size: %d", (int)cloud->size());
		// Some of the points in the cloud have nan or inf values, need to strip
		// those to avoid errors. This is only true for the intermediate clouds
		for (size_t i = 0; i < cloud->points.size(); i++) {
		    if (std::isnan(cloud->points[i].x) || std::isinf(cloud->points[i].x)
			|| std::isnan(cloud->points[i].y) || std::isinf(cloud->points[i].y)
			|| std::isnan(cloud->points[i].z) || std::isinf(cloud->points[i].z)){
			continue;
		    }
		    cloud_xyz->points[i].x = cloud->points[i].x;
		    cloud_xyz->points[i].y = cloud->points[i].y;
		    cloud_xyz->points[i].z = cloud->points[i].z;
		    added++;
		}
		ROS_INFO("Total invalid points: %d", ((int)cloud->size()) - added);
    
		cloud_xyz->points.resize(added);
        
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors(new pcl::PointCloud<pcl::ShapeContext1980>());

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud_xyz);
		sor.setLeafSize(0.05, 0.05, 0.05);
		sor.filter(*cloud_downsampled);

		ROS_INFO("Downsampled size: %d", (int)cloud_downsampled->size());
	
		pcl::UniqueShapeContext<pcl::PointXYZ, pcl::ShapeContext1980, pcl::ReferenceFrame> usc;
		// Compute the descriptors using all points in the main cloud
		usc.setSearchSurface(cloud_xyz);
		// Use the downsampled cloud to decide at which points to compute the descriptors
		usc.setInputCloud(cloud_downsampled);
		// Search radius, to look for neighbors. It will also be the radius of the support sphere.
		usc.setRadiusSearch(0.05);
		// The minimal radius value for the search sphere, to avoid being too sensitive
		// in bins close to the center of the sphere.
		usc.setMinimalRadius(0.05 / 10.0);
		// Radius used to compute the local point density for the neighbors
		// (the density is the number of points within that radius).
		usc.setPointDensityRadius(0.05 / 5.0);
		// Set the radius to compute the Local Reference Frame.
		usc.setLocalRadius(0.05);

		ROS_INFO("Computing descriptors.");
		usc.compute(*descriptors);
		ROS_INFO("Done.");
		pcl::PCDWriter writer;
		std::string featureOutFile = SysUtil::cleanDirPath(outPath) + "/features/"
		    + SysUtil::removeExtension(cloudFile) + "_usc.pcd";
		ROS_INFO("Writing computed features to %s", featureOutFile.c_str());
		writer.write<pcl::ShapeContext1980>(featureOutFile, *descriptors, true);

		// output the points at which the descriptors were computed so that they
		// can be used later
		std::string pointOutFile = SysUtil::cleanDirPath(outPath) + "/features/"
		    + SysUtil::removeExtension(cloudFile) + "_usc_points.pcd";
		ROS_INFO("Writing feature computation points to %s", pointOutFile.c_str());
		writer.write<pcl::PointXYZ>(pointOutFile, *cloud_downsampled, true);
	    } else {
		ROS_ERROR("%s is not a valid feature type.", featureType.c_str());
		exit(1);
	    }

	    //pcl::PointCloud<pcl::SHOT352>::Ptr descriptorsshot(new pcl::PointCloud<pcl::SHOT352>());
	    // std::vector<pcl::SHOT352> desc(5);
	    // for (int i = 0; i < 5; i++) {
	    // 	for (int j = 0; j < 352; j++) {
	    // 	    desc[i].descriptor[j] = j + i;
	    // 	}
	    // }

	    // pcl::PointCloud<pcl::SHOT352>::Ptr queryDescriptors(new pcl::PointCloud<pcl::SHOT352>());
	    // queryDescriptors->push_back(desc[0]);

	    // pcl::PointCloud<pcl::SHOT352>::Ptr targetDescriptors(new pcl::PointCloud<pcl::SHOT352>());
	    // for (int i = 0; i < 4; i++) {
	    // 	targetDescriptors->push_back(desc[i+1]);
	    // }

	    // pcl::PCDWriter writer;

	    // // create the directory for output if it has not already been created
	    // if (!SysUtil::makeDirs(outPath + "/features")){
	    // 	std::cout << "Could not write point clouds to output directory." << std::endl;
	    // 	perror("Error message");
	    // 	exit(1);
	    // }
    
	    // writer.write<pcl::SHOT352>(SysUtil::fullDirPath(outPath) + "/features/targetcloud.pcd", *targetDescriptors, true);
	    // writer.write<pcl::SHOT352>(SysUtil::fullDirPath(outPath) + "/features/querycloud.pcd", *queryDescriptors, true);

	}

	
    } // namespace featureExtraction
} // namespace objsearch


int main(int argc, char *argv[]) {
    objsearch::featureExtraction::FeatureExtractor fe(argc, argv);
    return 0;
}
