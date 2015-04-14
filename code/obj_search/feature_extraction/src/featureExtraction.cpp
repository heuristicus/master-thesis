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
	    ROSUtil::getParam(handle, "/feature_extraction/input_cloud", cloudFile_);
	    ROSUtil::getParam(handle, "/obj_search/processed_data_dir", processedDir_);
	    ROSUtil::getParam(handle, "/obj_search/raw_data_dir", rawDir_);
	    // If the given cloud file corresponds to a file in the processed data directory,
	    // extract the remaining directories in the path of the file so that the
	    // data can be put into the output directory with the same path.
	    if (cloudFile_.compare(0, processedDir_.size(), processedDir_) == 0){
		dataSubDir_ = SysUtil::trimPath(std::string(cloudFile_, processedDir_.size()), 1);
	    } else if (cloudFile_.compare(0, rawDir_.size(), rawDir_) == 0) {
		dataSubDir_ = SysUtil::trimPath(std::string(cloudFile_, rawDir_.size()), 1);
	    }

	    ROSUtil::getParam(handle, "/feature_extraction/output_dir", outDir_);
	    // If output is not specified, set the output directory to be the processed
	    // data directory specified by the global parameters.
	    if (std::string("NULL").compare(outDir_) == 0) {
		outDir_ = processedDir_;
	    }

	    // The output path for processed clouds is the subdirectory combined
	    // with the top level output directory. If dataSubDir_ is not
	    // initialised, then clouds are simply output to the top level
	    // output directory
	    outPath_ = SysUtil::combinePaths(outDir_, dataSubDir_);

	    ROSUtil::getParam(handle, "/feature_extraction/feature_type", featureType_);
	    ROSUtil::getParam(handle, "/feature_extraction/feature_selection", featureSelection_);
	    
	    ROSUtil::getParam(handle, "/feature_extraction/downsample_leaf_size", downsampleLeafSize_);

	    // SHOT
	    ROSUtil::getParam(handle, "/feature_extraction/shot_radius", shotRadius_);

	    // USC
	    ROSUtil::getParam(handle, "/feature_extraction/usc_radius", uscRadius_);
	    ROSUtil::getParam(handle, "/feature_extraction/usc_minimal_radius", uscMinRadius_);
	    ROSUtil::getParam(handle, "/feature_extraction/usc_density_radius", uscDensityRadius_);
	    ROSUtil::getParam(handle, "/feature_extraction/usc_local_radius", uscLocalRadius_);
	    
	    extractFeatures();
	}

	void FeatureExtractor::extractFeatures(){
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloudFile_, *cloud) != -1){
		ROS_INFO("Loaded cloud from %s", cloudFile_.c_str());
	    } else {
		ROS_INFO("Could not load cloud from %s", cloudFile_.c_str());
		exit(1);
	    }

	    // Define points at which descriptors should be computed.
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr descriptorLocations(new pcl::PointCloud<pcl::PointXYZRGB>());
	    if (featureSelection_.compare("uniform") == 0) {
		// Get points uniformly across the space using a voxel grid,
		// ignoring regions which have no points in them.
		ROS_INFO("Using uniform feature selection with leaf size %f.", downsampleLeafSize_);
		pcl::VoxelGrid<pcl::PointXYZRGB> vgrid;
		vgrid.setInputCloud(cloud);
		vgrid.setLeafSize(downsampleLeafSize_, downsampleLeafSize_, downsampleLeafSize_);
		vgrid.filter(*descriptorLocations);
	    } else {
		ROS_INFO("Unknown feature selection method %s", featureSelection_.c_str());
		exit(1);
	    }

	    ROS_INFO("Number of points to compute features at: %d", (int)descriptorLocations->size());

	    // get lowercase for the feature type to allow lowercase input as well as
	    // uppercase, because laziness
	    std::transform(featureType_.begin(), featureType_.end(), featureType_.begin(), ::tolower);
    
	    if (featureType_.compare("shot") == 0) {
		// load the cloud of normals. Should find a better way of
		// distinguishing between intermediate and complete clouds
		std::string normFile = SysUtil::trimPath(cloudFile_, 1) + '/';
		if (SysUtil::trimPath(cloudFile_, -1)[0] == '0') { // intermediate clouds start with zero
		    // intermediate has 4 digits followed by underscore
		    normFile += std::string(SysUtil::trimPath(cloudFile_, -1), 0, 5) + "normCloud.pcd";
		} else {
		    normFile += "normCloud.pcd";
		}

		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		if (pcl::io::loadPCDFile<pcl::Normal>(normFile, *normals) != -1){
		    ROS_INFO("Loaded cloud from %s", normFile.c_str());
		} else {
		    ROS_INFO("Could not load cloud from %s", normFile.c_str());
		    exit(1);
		}

		pcl::PointIndices::Ptr nanIndices(new pcl::PointIndices());
		
		for (size_t i = 0; i < normals->size(); i++) {
		    if (std::isnan(normals->points[i].data_c[0])
			|| std::isnan(normals->points[i].data_c[1])
			|| std::isnan(normals->points[i].data_c[2])
			|| std::isnan(cloud->points[i].x)
			|| std::isnan(cloud->points[i].y)
			|| std::isnan(cloud->points[i].z)){
			nanIndices->indices.push_back(i);
		    }
		}

		ROS_INFO("nan count: %d", (int)nanIndices->indices.size());
		ROS_INFO("cloud before filter: %d", (int)cloud->size());
		ROS_INFO("normals before filter: %d", (int)normals->size());
                // Modify the main cloud and the normal cloud to remove any
		// points which have nan values in the normals and will affect
		// the shot computation
		pcl::ExtractIndices<pcl::PointXYZRGB> exRGB;
		exRGB.setInputCloud(cloud);
		exRGB.setIndices(nanIndices);
		exRGB.setNegative(true); // extract non-nan indices
		exRGB.filter(*cloud);
		pcl::ExtractIndices<pcl::Normal> exNorm;
		exNorm.setInputCloud(normals);
		exNorm.setIndices(nanIndices);
		exNorm.setNegative(true); // extract non-nan indices
		exNorm.filter(*normals);

		ROS_INFO("cloud after filter: %d", (int)cloud->size());
		ROS_INFO("normals after filter: %d", (int)normals->size());

		pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

		pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
		shot.setInputCloud(descriptorLocations);
		shot.setSearchSurface(cloud);
		shot.setInputNormals(normals);
		
		// The radius that defines which of the keypoint's neighbors are
		// described. If too large, there may be clutter, and if too
		// small, not enough points may be found.
		shot.setRadiusSearch(shotRadius_);

		ROS_INFO("Computing descriptors.");
 		shot.compute(*descriptors);
		ROS_INFO("Done.");

		writeData<pcl::SHOT352, pcl::PointXYZRGB>(descriptors, descriptorLocations);
	    } else if (featureType_.compare("usc") == 0) {
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

		pcl::PointCloud<pcl::PointXYZ>::Ptr descriptorLocations_xyz(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(*descriptorLocations, *descriptorLocations_xyz);
		    
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors(new pcl::PointCloud<pcl::ShapeContext1980>());

		pcl::UniqueShapeContext<pcl::PointXYZ, pcl::ShapeContext1980, pcl::ReferenceFrame> usc;
		// Compute the descriptors using all points in the main cloud
		usc.setSearchSurface(cloud_xyz);
		// Use the downsampled cloud to decide at which points to compute the descriptors
		usc.setInputCloud(descriptorLocations_xyz);
		// Search radius, to look for neighbors. It will also be the radius of the support sphere.
		usc.setRadiusSearch(uscRadius_);
		// The minimal radius value for the search sphere, to avoid being too sensitive
		// in bins close to the center of the sphere.
		usc.setMinimalRadius(uscMinRadius_);
		// Radius used to compute the local point density for the neighbors
		// (the density is the number of points within that radius).
		usc.setPointDensityRadius(uscDensityRadius_);
		// Set the radius to compute the Local Reference Frame.
		usc.setLocalRadius(uscLocalRadius_);

		ROS_INFO("Computing descriptors.");
		usc.compute(*descriptors);
		ROS_INFO("Done.");
		writeData<pcl::ShapeContext1980, pcl::PointXYZ>(descriptors, descriptorLocations_xyz);
	    } else {
		ROS_ERROR("%s is not a valid feature type.", featureType_.c_str());
		exit(1);
	    }
	}

	template<typename DescType, typename PointType>
	void FeatureExtractor::writeData(const typename pcl::PointCloud<DescType>::Ptr& descriptors,
					 const typename pcl::PointCloud<PointType>::Ptr& points){
	    if (!SysUtil::makeDirs(SysUtil::cleanDirPath(outPath_) + "/features/")){
		ROS_INFO("Could not create output directory.");
	    }
	    pcl::PCDWriter writer;
	    std::string featureOutFile = SysUtil::cleanDirPath(outPath_) + "/features/"
		+ SysUtil::removeExtension(cloudFile_) + "_shot.pcd";
	    ROS_INFO("Writing computed features to %s", featureOutFile.c_str());
	    writer.write<DescType>(featureOutFile, *descriptors, true);

	    // output the points at which the descriptors were computed so that they
	    // can be used later
	    std::string pointOutFile = SysUtil::cleanDirPath(outPath_) + "/features/"
		+ SysUtil::removeExtension(cloudFile_) + "_shot_points.pcd";
	    ROS_INFO("Writing feature computation points to %s", pointOutFile.c_str());
	    writer.write<PointType>(pointOutFile, *points, true);
	}

	
    } // namespace featureExtraction
} // namespace objsearch


int main(int argc, char *argv[]) {
    objsearch::featureExtraction::FeatureExtractor fe(argc, argv);
    return 0;
}
