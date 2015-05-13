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
	    ROSUtil::getParam(handle, "/feature_extraction/output_dir", outDir_);


	    ROSUtil::getParam(handle, "/feature_extraction/feature_type", featureType_);
	    ROSUtil::getParam(handle, "/feature_extraction/feature_selection", interestType_);
	    
	    ROSUtil::getParam(handle, "/feature_extraction/downsample_leaf_size", downsampleLeafSize_);

	    // ISS
	    ROSUtil::getParam(handle, "/feature_extraction/iss_salient_mult", issSalientMult_);
	    ROSUtil::getParam(handle, "/feature_extraction/iss_nonmax_mult", issNonMaxMult_);
	    ROSUtil::getParam(handle, "/feature_extraction/iss_min_neighbours", issMinNeighbours_);
	    ROSUtil::getParam(handle, "/feature_extraction/iss_thresh21", issThreshold21_);
	    ROSUtil::getParam(handle, "/feature_extraction/iss_thresh32", issThreshold32_);

	    // SUSAN
	    ROSUtil::getParam(handle, "/feature_extraction/susan_nonmax", susanNonMax_);
	    ROSUtil::getParam(handle, "/feature_extraction/susan_radius", susanRadius_);
	    ROSUtil::getParam(handle, "/feature_extraction/susan_dist_thresh", susanDistThresh_);
	    ROSUtil::getParam(handle, "/feature_extraction/susan_ang_thresh", susanAngularThresh_);
	    ROSUtil::getParam(handle, "/feature_extraction/susan_intensity_thresh", susanIntensityThresh_);

	    // harris
	    ROSUtil::getParam(handle, "/feature_extraction/harris_nonmax", harrisNonMax_);
	    ROSUtil::getParam(handle, "/feature_extraction/harris_radius", harrisRadius_);
	    ROSUtil::getParam(handle, "/feature_extraction/harris_thresh", harrisThreshold_);
	    ROSUtil::getParam(handle, "/feature_extraction/harris_refine", harrisRefine_);

	    // SIFT
	    ROSUtil::getParam(handle, "/feature_extraction/sift_min_scale", siftMinScale_);
	    ROSUtil::getParam(handle, "/feature_extraction/sift_octaves", siftOctaves_);
	    ROSUtil::getParam(handle, "/feature_extraction/sift_octave_scales", siftOctaveScales_);
	    ROSUtil::getParam(handle, "/feature_extraction/sift_min_contrast", siftMinContrast_);
	    
	    // SHOT
	    ROSUtil::getParam(handle, "/feature_extraction/shot_radius", shotRadius_);

	    // USC
	    ROSUtil::getParam(handle, "/feature_extraction/usc_radius", uscRadius_);
	    ROSUtil::getParam(handle, "/feature_extraction/usc_minimal_radius", uscMinRadius_);
	    ROSUtil::getParam(handle, "/feature_extraction/usc_density_radius", uscDensityRadius_);
	    ROSUtil::getParam(handle, "/feature_extraction/usc_local_radius", uscLocalRadius_);

	    // FPFH
	    ROSUtil::getParam(handle, "/feature_extraction/fpfh_radius", fpfhRadius_);
	    // PFH
	    ROSUtil::getParam(handle, "/feature_extraction/pfh_radius", pfhRadius_);
	    // PFHRGB
	    ROSUtil::getParam(handle, "/feature_extraction/pfhrgb_radius", pfhrgbRadius_);
	    
	    
	    std::string match;
	    ROSUtil::getParam(handle, "/feature_extraction/match", match);


	    std::vector<std::string> roomFiles;
	    std::string dataOutput;
	    std::string timeNow = sysutil::getDateTimeString();
	    initPaths(cloudFile_);
	    
	    // room files are different depending on what input was received -
	    // directory, file, or directory with match string
	    if (sysutil::isDir(cloudFile_)) {
		if (match.compare("NULL") == 0) {
		    roomFiles = sysutil::listFilesWithString(cloudFile_, "nonPlanes.pcd", true);
		} else { // otherwise, match the string and process those files
		    roomFiles = sysutil::listFilesWithString(cloudFile_, match, true);
		}
		dataOutput = sysutil::fullDirPath(outPath_) + "featureparams_" + timeNow + ".yaml";
	    } else { // is a file, so just process that
		dataOutput = sysutil::fullDirPath(outPath_) + "featureparams_" + timeNow + ".yaml";
		roomFiles.push_back(cloudFile_);
	    }

	    if (roomFiles.size() == 0) {
		ROS_INFO("No matches for %s", match.c_str());
		exit(1);
	    }

	    // Dump parameters used for this run
	    // dangerous, but otherwise annoying to output all parameters individually.
	    std::string command("rosparam dump " + dataOutput);
	    ROS_INFO("Caling system with command %s", command.c_str());
	    system(command.c_str());

	    // print some information about what files will be processed
	    size_t i;
	    for (i = 0; i < 10 && i < roomFiles.size(); i++) {
		ROS_INFO("%s", roomFiles[i].c_str());
	    }
	    if (i >= 10) {
		ROS_INFO("And more...");
	    }

	    bool append = false;
	    std::string dataFile = sysutil::fullDirPath(sysutil::trimPath(dataOutput, 1))
		+ "featuredata_" + timeNow + ".txt";
	    for (auto it = roomFiles.begin(); it != roomFiles.end(); it++) {
		ROS_INFO("Extracting features from cloud %d of %d",
			 (int)(it - roomFiles.begin()) + 1, (int)roomFiles.size());
		cloudFile_ = *it;
		initPaths(cloudFile_); // update output paths
		FeatureInfo info = extractFeatures();
		writeInfo(dataFile, info, append);
		append = true;
	    }
	}

	void FeatureExtractor::writeInfo(std::string outPath, FeatureInfo info, bool append) {
	    std::ofstream file;
	    if (append){
		file.open(outPath, std::ios::app);
	    } else {
		file.open(outPath);
	    }

	    // write the column headers: filename for the information output,
	    // number of points before modification, number of points selected
	    // for feature computation, time taken for selection, time taken for
	    // feature computation
	    if (!append) {
		// if not appending, put headers to the columns and the
		// directory/file the program was run on
		file << cloudFile_.c_str()
		     << "#filename n_pre n_feature t_select t_feature" << std::endl;
	    }

	    // output data from the struct
	    file << info.fname.c_str() << " " << info.originalSize << " " << info.featureSize
		 << " " << info.selectTime << " " << info.featureTime << std::endl;
	    file.close();
	}


	void FeatureExtractor::initPaths(std::string path) {
	    // if the path is a directory, don't want to trim anything off the end.
	    int toTrim = 1;
	    if (sysutil::isDir(path)) {
		toTrim = 0;
	    }
	    // If the given cloud file corresponds to a file in the processed data directory,
	    // extract the remaining directories in the path of the file so that the
	    // data can be put into the output directory with the same path.
	    if (path.compare(0, processedDir_.size(), processedDir_) == 0){
		dataSubDir_ = sysutil::trimPath(std::string(path, processedDir_.size()), toTrim);
	    } else if (path.compare(0, rawDir_.size(), rawDir_) == 0) {
		dataSubDir_ = sysutil::trimPath(std::string(path, rawDir_.size()), toTrim);
	    }
	    // If output is not specified, set the output directory to be the processed
	    // data directory specified by the global parameters.
	    if (std::string("NULL").compare(outDir_) == 0) {
		outDir_ = processedDir_;
	    }

	    // The output path for processed clouds is the subdirectory combined
	    // with the top level output directory. If dataSubDir_ is not
	    // initialised, then clouds are simply output to the top level
	    // output directory
	    outPath_ = sysutil::combinePaths(outDir_, dataSubDir_);
	}
	
	// from http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
	double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
	    double res = 0.0;
	    int n_points = 0;
	    int nres;
	    std::vector<int> indices(2);
	    std::vector<float> sqr_distances(2);
	    pcl::search::KdTree<pcl::PointXYZRGB> tree;
	    tree.setInputCloud(cloud);

	    for (size_t i = 0; i < cloud->size(); ++i) {
		if (!pcl_isfinite((*cloud)[i].x)) {
		    continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if(nres == 2) {
		    res += sqrt(sqr_distances[1]);
		    ++n_points;
		}
	    }
	    if (n_points != 0){
		res /= n_points;
	    }
	    return res;
	}

	/** 
	 * Convert the given XYZRGB cloud to XYZI. RGB converted to intensity by
	 * taking the average of the three channels. Automatically deals with
	 * NaN values.
	 * 
	 * @param in the cloud to convert to XYZI
	 * @param out This cloud will be populated with the points from the in
	 * cloud converted to intensity
	 */
	void rgbToIntensity(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in,
			    pcl::PointCloud<pcl::PointXYZI>::Ptr& out) {
	    out->resize(in->size());
	    int nancount = 0;
	    for (int i = 0; i < (int)in->size(); i++) {
		if (std::isnan(in->points[i].x)
		    || std::isnan(in->points[i].y)
		    || std::isnan(in->points[i].z)){
		    nancount++;
		    continue;
		}
		out->points[i].x = in->points[i].x;
		out->points[i].y = in->points[i].y;
		out->points[i].z = in->points[i].z;
		out->points[i].intensity = pclutil::getRGBIntensityBasic(in->points[i].r, in->points[i].g, in->points[i].b);
	    }
	    // once done, resize the cloud so that there are no empty spaces
	    // where nan valued points would have been.
	    out->resize(in->size() - nancount);
	}

	/** 
	 * Convert the given XYZI cloud to XYZRGB. The intensity value is
	 * ignored, so all points have rgb values of 0.
	 * 
	 * @param in the cloud to convert to XYZRGB
	 * @param out This cloud will be populated with the points from the in
	 * cloud
	 */
	void intensityToRGB(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
			    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out) {
	    out->resize(in->size());
	    for (int i = 0; i < (int)in->size(); i++) {
		out->points[i].x = in->points[i].x;
		out->points[i].y = in->points[i].y;
		out->points[i].z = in->points[i].z;
	    }
	}

	FeatureExtractor::FeatureInfo FeatureExtractor::extractFeatures(){
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloudFile_, *cloud) != -1){
		ROS_INFO("Loaded cloud from %s", cloudFile_.c_str());
	    } else {
		ROS_INFO("Could not load cloud from %s", cloudFile_.c_str());
		exit(1);
	    }
	    FeatureInfo info;
	    info.fname = cloudFile_;
	    info.originalSize = cloud->size();

	    // Define points at which descriptors should be computed.
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr descriptorLocations(new pcl::PointCloud<pcl::PointXYZRGB>());
	    
	    ros::Time selectStart = ros::Time::now();
	    if (interestType_.compare("uniform") == 0) {
		// Get points uniformly across the space using a voxel grid,
		// ignoring regions which have no points in them.
		ROS_INFO("Using uniform feature selection with leaf size %f.", downsampleLeafSize_);
		pcl::VoxelGrid<pcl::PointXYZRGB> vgrid;
		vgrid.setInputCloud(cloud);
		vgrid.setLeafSize(downsampleLeafSize_, downsampleLeafSize_, downsampleLeafSize_);
		vgrid.filter(*descriptorLocations);
	    } else if (interestType_.compare("iss") == 0) {
		ROS_INFO("Using ISS feature selection.");
		pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss;
		iss.setInputCloud(cloud);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		iss.setSearchMethod(kdtree);
		double resolution = computeCloudResolution(cloud);
		ROS_INFO("Cloud resolution is %f", resolution);
		// see launch file for description of these parameters
		iss.setSalientRadius(issSalientMult_ * resolution);
		iss.setNonMaxRadius(issNonMaxMult_ * resolution);
		iss.setMinNeighbors(issMinNeighbours_);
		iss.setThreshold21(issThreshold21_);
		iss.setThreshold32(issThreshold32_);

		iss.compute(*descriptorLocations);
	    } else if (interestType_.compare("harris") == 0) {
		ROS_INFO("Using Harris3D feature selection.");
		loadNormals(normals); // might end up doing this again later on
				      // if using certain types of features

		// Need to convert to xyzi from xyzrgb, harris doesn't use rgb
		pcl::PointCloud<pcl::PointXYZI>::Ptr intensity(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr intensityOut(new pcl::PointCloud<pcl::PointXYZI>);
		rgbToIntensity(cloud, intensity);
		
		pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> harris;
		harris.setInputCloud(intensity);
		harris.setNormals(normals);
		harris.setThreshold(harrisThreshold_);
		harris.setRadius(harrisRadius_);
		harris.setNonMaxSupression(harrisNonMax_);
		harris.setRefine(harrisRefine_);

		harris.compute(*intensityOut);
		
		intensityToRGB(intensityOut, descriptorLocations);
	    } else if (interestType_.compare("susan") == 0) {
		ROS_INFO("Using SUSAN feature selection.");
		loadNormals(normals); // might end up doing this again later on
				      // if using certain types of features
		pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB> susan;
		susan.setInputCloud(cloud);
		susan.setNormals(normals);
		susan.setNonMaxSupression(susanNonMax_);
		susan.setRadius(susanRadius_);
		susan.setDistanceThreshold(susanDistThresh_);
		susan.setAngularThreshold(susanAngularThresh_);
		susan.setIntensityThreshold(susanIntensityThresh_);

		susan.compute(*descriptorLocations);
	    } else if (interestType_.compare("sift") == 0) {
		ROS_INFO("Using SIFT feature selection.");
		// Need to convert to xyzi from xyzrgb, harris doesn't use rgb
		pcl::PointCloud<pcl::PointXYZI>::Ptr intensity(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr intensityOut(new pcl::PointCloud<pcl::PointXYZI>);
		rgbToIntensity(cloud, intensity);
		
		pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZI> sift;
		sift.setInputCloud(intensity);
		sift.setScales(siftMinScale_, siftOctaves_, siftOctaveScales_);
		sift.setMinimumContrast(siftMinContrast_);
		sift.compute(*intensityOut);

		intensityToRGB(intensityOut, descriptorLocations);
	    } else {
		ROS_INFO("Unknown feature selection method %s", interestType_.c_str());
		exit(1);
	    }
	    info.selectTime = (ros::Time::now() - selectStart).toSec();
	    info.featureSize = descriptorLocations->size();

	    ROS_INFO("Number of points to compute features at: %d", (int)descriptorLocations->size());

	    // get lowercase for the feature type to allow lowercase input as well as
	    // uppercase, because laziness
	    std::transform(featureType_.begin(), featureType_.end(), featureType_.begin(), ::tolower);

	    ros::Time featureStart = ros::Time::now();
	    if (featureType_.find("shot") != std::string::npos) { // two different shot descriptors with same preprocess

		loadNormals(normals);

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
		// ROS_INFO("cloud before filter: %d", (int)cloud->size());
		// ROS_INFO("normals before filter: %d", (int)normals->size());
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

		// ROS_INFO("cloud after filter: %d", (int)cloud->size());
		// ROS_INFO("normals after filter: %d", (int)normals->size());

		if (featureType_.compare("shot") == 0) {
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
		} else if (featureType_.compare("shotcolor") == 0) {
		    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT1344>());

		    pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot;
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

		    writeData<pcl::SHOT1344, pcl::PointXYZRGB>(descriptors, descriptorLocations);		    
		} else {
		    ROS_INFO("Unknown SHOT descriptor type %s", featureType_.c_str());
		    exit(1);
		}
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
	    } else if (featureType_.find("pfh") != std::string::npos) { // point feature histogram, multiple types
		loadNormals(normals);
		
		if (featureType_.compare("pfh") == 0) {
		    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
		    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
		    
		    pfh.setInputCloud(descriptorLocations);
		    pfh.setSearchSurface(cloud);
		    pfh.setInputNormals(normals);
		    pfh.setSearchMethod(kdtree);
		    pfh.setRadiusSearch(pfhRadius_);
		    ROS_INFO("Starting feature computation");
		    pfh.compute(*descriptors);
		    ROS_INFO("Feature computation done");
		    
		    writeData<pcl::PFHSignature125, pcl::PointXYZRGB>(descriptors, descriptorLocations);
		} else if (featureType_.compare("fpfh") == 0) {
		    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
		    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
		    
		    fpfh.setInputCloud(descriptorLocations);
		    fpfh.setSearchSurface(cloud);
		    fpfh.setInputNormals(normals);
		    fpfh.setSearchMethod(kdtree);
		    fpfh.setRadiusSearch(fpfhRadius_);
		    ROS_INFO("Starting feature computation");
		    fpfh.compute(*descriptors);
		    ROS_INFO("Feature computation done");
		    
		    writeData<pcl::FPFHSignature33, pcl::PointXYZRGB>(descriptors, descriptorLocations);
		} else if (featureType_.compare("pfhrgb") == 0) {
		    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptors(new pcl::PointCloud<pcl::PFHRGBSignature250>());
		    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		    pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> pfhrgb;
		    
		    pfhrgb.setInputCloud(descriptorLocations);
		    pfhrgb.setSearchSurface(cloud);
		    pfhrgb.setInputNormals(normals);
		    pfhrgb.setSearchMethod(kdtree);
		    pfhrgb.setRadiusSearch(pfhrgbRadius_);
		    ROS_INFO("Starting feature computation");
		    pfhrgb.compute(*descriptors);
		    ROS_INFO("Feature computation done");
		    
		    writeData<pcl::PFHRGBSignature250, pcl::PointXYZRGB>(descriptors, descriptorLocations);
		} else {
		    ROS_INFO("Unknown pfh feature type %s", featureType_.c_str());
		    exit(1);
		}
	    } else {
		ROS_ERROR("%s is not a valid feature type.", featureType_.c_str());
		exit(1);
	    }
	    info.featureTime = (ros::Time::now() - featureStart).toSec();
	    return info;
	}

	template<typename DescType, typename PointType>
	void FeatureExtractor::writeData(const typename pcl::PointCloud<DescType>::Ptr& descriptors,
					 const typename pcl::PointCloud<PointType>::Ptr& points){
	    if (!sysutil::makeDirs(sysutil::cleanDirPath(outPath_) + "/features/")){
		ROS_INFO("Could not create output directory.");
	    }
	    pcl::PCDWriter writer;
	    std::string featureOutFile = sysutil::cleanDirPath(outPath_) + "/features/"
		+ sysutil::removeExtension(cloudFile_) + "_" + featureType_ + "_" + interestType_ + ".pcd";
	    ROS_INFO("Writing computed features to %s", featureOutFile.c_str());
	    writer.write<DescType>(featureOutFile, *descriptors, true);

	    // output the points at which the descriptors were computed so that they
	    // can be used later
	    std::string pointOutFile = sysutil::cleanDirPath(outPath_) + "/features/"
		+ sysutil::removeExtension(cloudFile_) + "_" + featureType_ + "_" + interestType_ +"_points.pcd";
	    ROS_INFO("Writing feature computation points to %s", pointOutFile.c_str());
	    writer.write<PointType>(pointOutFile, *points, true);
	}

	void FeatureExtractor::loadNormals(pcl::PointCloud<pcl::Normal>::Ptr& normals){
	    // load the cloud of normals. Should find a better way of
	    // distinguishing between intermediate and complete clouds
	    std::string normFile = sysutil::trimPath(cloudFile_, 1) + '/';
	    if (sysutil::trimPath(cloudFile_, -1)[0] == '0') { // intermediate clouds start with zero
		// intermediate has 4 digits followed by underscore
		normFile += std::string(sysutil::trimPath(cloudFile_, -1), 0, 5) + "normCloud.pcd";
	    } else if (cloudFile_.find("label") != std::string::npos) { // annotation clouds contain the text "label"
		normFile += sysutil::removeExtension(cloudFile_) + "_normals.pcd";
	    } else {
		normFile += "normCloud.pcd";
	    }


	    if (pcl::io::loadPCDFile<pcl::Normal>(normFile, *normals) != -1){
		ROS_INFO("Loaded cloud from %s", normFile.c_str());
	    } else {
		ROS_INFO("Could not load cloud from %s", normFile.c_str());
		exit(1);
	    }
	}

	
    } // namespace featureExtraction
} // namespace objsearch


int main(int argc, char *argv[]) {
    objsearch::featureExtraction::FeatureExtractor fe(argc, argv);
    return 0;
}
