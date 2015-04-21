/**
 * @file   planeTest.cpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Mon Mar  9 14:06:36 2015
 * 
 * @brief  Some prototype code for doing preprocessing on point clouds.
 */
#include "preprocess.hpp"

namespace objsearch {
    namespace preprocessing {

	// Declare static members of the class
	const std::string PreprocessRoom::intermediatePrefix = "intermediate_cloud";
	const std::string PreprocessRoom::completeCloudName = "complete_cloud.pcd";

	/** 
	 * Constructor for the preprocessing node class. Uses parameters
	 * received from the parameter server to set up internal attributes.
	 * Once the object has been constructed, it calls functions to process
	 * the given point cloud.
	 * 
	 * @param argc Number of input arguments from main
	 * @param argv Values of input arguments from main
	 */
	PreprocessRoom::PreprocessRoom(int argc, char* argv[]){
	    ros::init(argc, argv, "preprocess");
	    ros::NodeHandle handle;

	    // Retrieve the path to the cloud to be processed
	    ROSUtil::getParam(handle, "/preprocess/cloud", cloudPath_);
	    if (!SysUtil::isFile(cloudPath_)){
		ROS_ERROR("%s does not exist.", cloudPath_.c_str());
		exit(1);
	    }
	    
	    // remove the filename from the end of the path to get the directory
	    cloudDir_ = SysUtil::trimPath(cloudPath_, 1);
	    // remove the front of the path to get just the filename
	    cloudFile_ = SysUtil::trimPath(cloudPath_, -1);
	    
	    // Construct the filenames for the XML file containing data
	    roomXML_ = SysUtil::fullDirPath(cloudDir_) + "room.xml";

	    if (cloudFile_.compare(completeCloudName) == 0) {
		type_ = CloudType::FULL;
		// trivial case
	    } else if (cloudFile_.find(intermediatePrefix) == 0) {
		type_ = CloudType::INTERMEDIATE;
		// the filename stars with the expected prefix for an
		// intermediate cloud
		// prefix the output by the number of the cloud. The files are
		// all in a standard format: intermediate_cloudxxxx.pcd, where
		// xxxx is a padded integer
		std::string cloudNum_Str(cloudFile_, intermediatePrefix.length(), 4);
		outPrefix_ = cloudNum_Str + "_";
		cloudNum_ = std::stoi(cloudNum_Str);
		
		// need to do additional work to ensure that cloudNum_ refers to
		// the position of the requested file in a vector where clouds
		// are pushed onto the back. If not all the intermediate cloud
		// files are present in cloudDir_, then the index of the cloud is
		// likely to be different from the number of the cloud in the
		// filename.
		std::vector<std::string> files = SysUtil::listDir(cloudDir_).files;
		// listing is unsorted, need things in order to get the correct index
		std::sort(files.begin(), files.end());
		cloudNum_ = 0; // reset the cloud number to fill with the correct index
		for (auto it = files.begin(); it != files.end(); ++it) {
		    // check that the file we are looking at is an intermediate
		    // cloud. The files have their whole path - trim them so we
		    // only look at the filename itself
		    std::string fname = SysUtil::trimPath(*it, -1);
		    if (fname.find(intermediatePrefix) == 0){
			if (fname.compare(cloudFile_) == 0){
			    // if the entire filename matches, then we have the desired index, so break
			    break;
			}
			// otherwise, increment the index
			cloudNum_++;
		    }
		}
	    } else {
		type_ = CloudType::OTHER;
	    }
	    
	    ROSUtil::getParam(handle, "/obj_search/raw_data_dir", dataPath_);

	    // If the given cloud file corresponds to a file in the raw data
	    // directory, extract the remaining directories in the path of the
	    // file so that the data can be put into the output directory with
	    // the same path. e.g. if raw_data_dir is set to /home/user/data and
	    // the input cloud is in /home/user/data/sets/set1/, then datasubdir
	    // will be /sets/set1.
	    if (cloudPath_.compare(0, dataPath_.size(), dataPath_) == 0){
		dataSubDir_ = std::string(cloudDir_, dataPath_.size());
	    } else {
		// if not in raw data, just output to the input directory
		dataSubDir_ = SysUtil::trimPath(cloudDir_, -1);
	    }

	    ROSUtil::getParam(handle, "/preprocess/output_dir", outDir_);
	    // If output is not specified, set the output directory to be the processed
	    // data directory specified by the global parameters.
	    if (std::string("NULL").compare(outDir_) == 0) {
		ROSUtil::getParam(handle, "/obj_search/processed_data_dir", outDir_);
	    }

	    // The output path for processed clouds is the subdirectory combined
	    // with the top level output directory. If dataSubDir_ is not
	    // initialised, then clouds are simply output to the top level
	    // output directory
	    outPath_ = SysUtil::combinePaths(outDir_, dataSubDir_);

	    ROSUtil::getParam(handle, "/preprocess/extract_planes", doExtractPlanes_);
	    ROSUtil::getParam(handle, "/preprocess/RANSAC_distance_threshold", ransacDistanceThresh_);
	    ROSUtil::getParam(handle, "/preprocess/RANSAC_iterations", ransacIterations_);
	    ROSUtil::getParam(handle, "/preprocess/planes_to_extract", planesToExtract_);
	    ROSUtil::getParam(handle, "/preprocess/min_plane_prop_complete", minPlanePropComplete_);
	    ROSUtil::getParam(handle, "/preprocess/min_plane_prop_intermediate", minPlanePropIntermediate_);
	    ROSUtil::getParam(handle, "/preprocess/min_plane_points_complete", minPlanePointsComplete_);
	    ROSUtil::getParam(handle, "/preprocess/min_plane_points_intermediate", minPlanePointsIntermediate_);
	    ROSUtil::getParam(handle, "/preprocess/plane_skip_limit", planeSkipLimit_);

	    ROSUtil::getParam(handle, "/preprocess/trim_cloud", doTrimCloud_);
	    ROSUtil::getParam(handle, "/preprocess/rotate_annotations", doRotateAnnotations_);
	    ROSUtil::getParam(handle, "/preprocess/floor_offset", floorOffset_);
	    ROSUtil::getParam(handle, "/preprocess/ceiling_offset", ceilingOffset_);
	    ROSUtil::getParam(handle, "/obj_search/floor_z", floorZ_);
	    ROSUtil::getParam(handle, "/obj_search/ceiling_z", ceilingZ_);

	    ROSUtil::getParam(handle, "/preprocess/compute_normals", doComputeNormals_);
	    ROSUtil::getParam(handle, "/preprocess/normal_radius", normalRadius_);
	    
	    ROSUtil::getParam(handle, "/preprocess/downsample", doDownsample_);
	    ROSUtil::getParam(handle, "/preprocess/downsample_leafsize", downsampleLeafSize_);
	    ROSUtil::getParam(handle, "/preprocess/downsample_increment", downsampleIncrement_);

	    ROS_INFO("Initialisation completed.");

	    preprocessCloud();
	}

	/** 
	 * Start the processing of the point cloud.
	 */
	void PreprocessRoom::preprocessCloud() {
	    ROS_INFO("Start processing");
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		originalCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	    tf::StampedTransform cloudTransform;

	    loadCloud(originalCloud, cloudTransform);

	    // create the directory for output if it has not already been created
	    if (!SysUtil::makeDirs(outPath_)){
		ROS_INFO("Could not create output directory %s.", outPath_.c_str());
		perror("Error message");
		exit(1);
	    }

	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr workingCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	    if (doDownsample_) {
		ROS_INFO("Points before downsample: %d", (int)originalCloud->size());
		pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		sor.setInputCloud(originalCloud);
		sor.setLeafSize(downsampleLeafSize_, downsampleLeafSize_,
				downsampleLeafSize_);
		sor.filter(*workingCloud);

		// Workaround to avoid issues with the "Leaf size is too small
		// for the input dataset. Integer indices would overflow."
		// warning which results in no downsampling. Try increasing the
		// leaf size until the downsampling works correctly.
		while (workingCloud->size() == originalCloud->size()) {
		    downsampleLeafSize_ += downsampleIncrement_; // increase by 0.5mm each loop
		    ROS_INFO("Retrying with leaf size %f", downsampleLeafSize_);
		    sor.setLeafSize(downsampleLeafSize_, downsampleLeafSize_,
				    downsampleLeafSize_);
		    sor.filter(*workingCloud);
		}
		
		ROS_INFO("Points after downsample: %d", (int)workingCloud->size());
	    } else {
		workingCloud = originalCloud;
	    }
	    ROS_INFO("workingCloud points: %d", (int)workingCloud->size());
	    
	    // non-dataset clouds have no transform information, so skip that step
	    if (type_ != CloudType::OTHER && doTrimCloud_) {
		ROS_INFO("Transforming and trimming cloud");
		transformAndRemoveFloorCeiling(workingCloud, cloudTransform);
		ROS_INFO("Cloud size after trim: %d", (int)workingCloud->size());
	    }

	    exit(1);
	    
	    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	    // need to compute normals before extracting planes
	    if (doComputeNormals_) {
		ROS_INFO("Computing normals");
		computeNormals(workingCloud, normals, cloudTransform);

		// pcl::concatenateFields(*workingCloud, *normals, *cloudWithNormals);

		// pcl::PCDWriter writer;
		// ROS_INFO("Outputting normals to %s", std::string(SysUtil::fullDirPath(outPath_) + "normCloud.pcd").c_str());
		// writer.write<pcl::Normal>(SysUtil::fullDirPath(outPath_) + "cloudWithNormals.pcd",
		// 			  *cloudWithNormals, true);
	    }
	    
	    if (doExtractPlanes_) {
		ROS_INFO("Extracting planes.");
		extractPlanes(workingCloud, normals);
		ROS_INFO("Cloud size after plane extraction: %d", (int)workingCloud->size());
		ROS_INFO("Normals size after plane extraction: %d", (int)normals->size());		
	    }

	    // Only save the normals cloud once it has also been pruned by the plane extraction
	    if (doComputeNormals_){
		pcl::PCDWriter writer;
		std::string outFile = SysUtil::fullDirPath(outPath_) + outPrefix_ + "normCloud.pcd";
		ROS_INFO("Outputting normals to %s", outFile.c_str());
		writer.write<pcl::Normal>(outFile, *normals, true);
	    }

	}

	/** 
	 * Load a pointcloud from the directory of interest, using the given XML
	 * file to extract information about the rotation of the room, if the
	 * cloud has an associated XML file. The cloud data is loaded into the
	 * cloud pointer passed to the function. Transformation data is put into
	 * the StampedTransform given; if there is none, it remains as it was.
	 * 
	 * @param cloud This pointer to a cloud will be populated with the
	 * merged cloud representing the room.
	 * @param cloudTransform This object will be populated with the
	 * transformation from the global frame to the local cloud frame, or
	 * left empty if there is no transform data.
	 */
	void PreprocessRoom::loadCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, // needs to be a reference to allow modification. Weird pointer type
				       tf::StampedTransform& cloudTransform) {
	    if (type_ == CloudType::OTHER){
		ROS_INFO("Reading cloud");
		pcl::PCDReader reader;
		reader.read(cloudPath_, *cloud);
	    } else {
		SimpleXMLParser<pcl::PointXYZRGB> parser;
		ROS_INFO("Parsing room XML.");
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData roomData = parser.loadRoomFromXML(roomXML_);
		ROS_INFO("Parse complete.");
		if (type_ == CloudType::FULL) {
		    ROS_INFO("Getting complete cloud");
		    cloud = roomData.completeRoomCloud->makeShared();
		    cloudTransform = roomData.vIntermediateRoomCloudTransforms[0];
		} else {
		    ROS_INFO("Getting intermediate cloud");
		    cloud = roomData.vIntermediateRoomClouds[cloudNum_]->makeShared();
		    cloudTransform = roomData.vIntermediateRoomCloudTransforms[cloudNum_];
		}
	    }
	}

	/** 
	 * The cloud we receive is translated and rotated relative to the global
	 * frame. It is possible to extract this from the gathered data using
	 * the XML provided with each room indicating the rotation of
	 * intermediate clouds. We use the rotation of the first intermediate
	 * cloud to translate and rotate the merged cloud into its actual
	 * position, and then threshold the z-axis values according to the
	 * height of the floor and ceiling (plus some offset) to remove points
	 * which make up those structures.
	 *
	 * @param cloud Cloud to transform and remove the floor and ceiling
	 * from.
	 * @param cloudTransform The transform needed to put the cloud into
	 * its position relative to the global reference frame. Ideally
	 * extracted from the XML data for the room and its intermediate clouds.
	 */
	void PreprocessRoom::transformAndRemoveFloorCeiling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
							    const tf::StampedTransform& cloudTransform){
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	    // This is the point at which the camera was while taking the images.
	    // tf::Vector3 origin = cloudTransform.getOrigin();
	    // std::cout << origin.getX() << ", " << origin.getY() << ", " << origin.getZ() << std::endl;
	    // Transform the cloud according to the given transform. After this
	    // point the x-y axis should be aligned with the floor, and the
	    // z-axis should point upwards.
	    pcl_ros::transformPointCloud(*cloud, *transformedCloud, cloudTransform);

	    // pcl::PointXYZRGB min;
	    // pcl::PointXYZRGB max;
	    // pcl::getMinMax3D(*transformedCloud, min, max);

	    // ROS_INFO("min: %f, %f, %f", min.x, min.y, min.z);
	    // ROS_INFO("max: %f, %f, %f", max.x, max.y, max.z);

	    // Use a passthrough filter to remove points beyond a specific
	    // threshold - the floor position plus an offset, and the ceiling
	    // position minus an offset. Positive z points towards the ceiling.
	    // The floor and ceiling heights are known, but we want to remove
	    // the floors and ceilings, so a little bit is added to make sure that happens.
	    pcl::PassThrough<pcl::PointXYZRGB> pass;
	    pass.setInputCloud(transformedCloud);
	    pass.setFilterFieldName("z");
	    // a little bit above the floor, and a little below the ceiling are our limits
	    pass.setFilterLimits(floorZ_ + floorOffset_, ceilingZ_ - ceilingOffset_);
	    // filter the values outside the thresholds and put the rest of the
	    // points back into the cloud.
	    pass.filter(*cloud);

	    pcl::PCDWriter writer;
	    ROS_INFO("Writing transformed cloud...");
	    writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(cloudDir_) + outPrefix_
					   + "transformedRoom.pcd", *transformedCloud, true);
	    ROS_INFO("Done");
	    ROS_INFO("Writing trimmed cloud...");
	    writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(cloudDir_) + outPrefix_
					   + "trimmedRoom.pcd", *cloud, true);

	    if (doRotateAnnotations_ && type_ == CloudType::FULL){ // only put the annotations into the full cloud reference frame
		std::vector<pclutil::AnnotatedCloud<pcl::PointXYZRGB> > annotations
		    = pclutil::getRawAnnotatedClouds<pcl::PointXYZRGB>(cloudDir_);
		
		for (size_t i = 0; i < annotations.size(); i++) {
		    // annotations start off in the same frame as the complete
		    // cloud, so just apply the same transform as is applied to
		    // everything else
		    pcl_ros::transformPointCloud(*(annotations[i].cloud), *transformedCloud, cloudTransform);

		    // get the file name
		    std::string nameRoot = SysUtil::trimPath(annotations[i].fname, -1);
		    // remove the extension and another bit of the annotation
		    // string preceding the label number to make the base string
		    // on top of which the actual label will be placed.
		    std::string base = std::string(nameRoot.begin(),
						   nameRoot.begin() + nameRoot.find_last_of('_'));

		    // no prefix for the annotations, they are the same whether
		    // the cloud input is intermediate or the complete cloud
		    writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(outPath_) + 
						   base + "_" + annotations[i].label + ".pcd",
						   *(transformedCloud), true);
		}
	    }
	    
	    ROS_INFO("Done");
	}

	/** 
	 * Once the room has been stripped of its floor and ceiling, remove
	 * other planes from the room. We aim to remove walls and other large
	 * flat surfaces from which we do not want to extract features. This
	 * process is done using a basic RANSAC procedure.
	 *
	 * @param cloud Pointer to the cloud from which planes are to be extracted.
	 * 
	 */
	void PreprocessRoom::extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
					   pcl::PointCloud<pcl::Normal>::Ptr& normals){
	    if (doComputeNormals_){
		pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
		seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
		seg.setInputNormals(normals);
		seg.setEpsAngle(0.25);
		extractPlanes(seg, cloud, normals);
	    } else {
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setModelType(pcl::SACMODEL_PLANE);
		extractPlanes(seg, cloud, normals);
	    }
	}
	
	template<typename SegmentationType>
	void PreprocessRoom::extractPlanes(SegmentationType& seg,
					   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
					   pcl::PointCloud<pcl::Normal>::Ptr& normals){
	    ROS_INFO("Extracting planes.");
	    ROS_INFO("Number of points in original cloud: %d", (int)cloud->size());
	    ROS_INFO("Number of normals in original cloud: %d", (int)normals->size());

	    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	    // Set up the segmentation to extract planes from the cloud using
	    // RANSAC with the parameters specified using the launch file
	    seg.setOptimizeCoefficients(true);
	    seg.setMethodType(pcl::SAC_RANSAC);
	    seg.setDistanceThreshold(ransacDistanceThresh_);
	    seg.setMaxIterations(ransacIterations_);

	    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	    // if just using the standard segmentation, this will be empty and
	    // nothing will happen within the loop
	    pcl::ExtractIndices<pcl::Normal> extractNormal;
	    extractNormal.setInputCloud(normals); // always using the same reference
	    extractNormal.setNegative(true);
	    
	    // Points will be removed from this cloud - at the end of the process it
	    // will contain all the points which were not extracted by the segmentation
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intermediateCloud(cloud);
	    // At each stage, the inliers of the plane model will be extracted to this
	    // cloud
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractedPlane(new pcl::PointCloud<pcl::PointXYZRGB>);
	    // All of the points that are extracted throughout the process will end up
	    // in this cloud
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr allPlanes(new pcl::PointCloud<pcl::PointXYZRGB>);
	    // The points which are not inliers to the plane will be placed into this
	    // cloud and then swapped into intermediateCloud
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr remainingPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	    pcl::PCDWriter writer;
	    ROS_INFO("Starting plane extraction.");
	    // Choose the larger of a specified number of points, or a
	    // proportion of the cloud size. This should mitigate fluctuating
	    // cloud sizes somewhat
	    int minPoints = 2000;
	    if (type_ == CloudType::INTERMEDIATE) {
		minPoints = std::max((int)(intermediateCloud->size() * minPlanePropIntermediate_), minPlanePointsIntermediate_);
	    } else if (type_ == CloudType::FULL) {
		minPoints = std::max((int)(intermediateCloud->size() * minPlanePropComplete_), minPlanePointsComplete_);
	    }
	    
	    int skipped = 0;
	    ROS_INFO("Minimum points per plane: %d", minPoints);
	    // keep going until the requested number of planes have been
	    // extracted, or the maximum number of skips in a row have occurred.
	    for (int i = 0; i < planesToExtract_ && skipped < planeSkipLimit_; i++) {
		ROS_INFO("Extracting plane %d", i + 1);
		seg.setInputCloud(intermediateCloud);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0) {
		    ROS_WARN("Could not estimate a planar model for the given dataset");
		    break;
		} else if (inliers->indices.size() < minPoints) {
		    ROS_INFO("Plane only had %d points, less than the specified minimum of %d - skipping extraction",
			     (int)inliers->indices.size(), minPoints);
		    skipped++;
		    continue;
		}
		
		// reset the number of skipped planes
		skipped = 0;

		ROS_INFO("Size of intermediate cloud: %d", (int)intermediateCloud->size());
		
		// Extract the inliers
		extract.setInputCloud(intermediateCloud);
		extract.setIndices(inliers);
		extract.setNegative(false); // Extract the points which are inliers
		extract.filter(*extractedPlane);
		ROS_INFO("Number of points on extracted plane: %d",
			 (int)extractedPlane->size());
		ROS_INFO("Number of points on all planes: %d",
			 (int)allPlanes->size());
		// Add the extracted inliers to the cloud of all planes
		*allPlanes += *extractedPlane; 
		ROS_INFO("Number of points after adding new plane: %d",
			 (int)allPlanes->size());

		writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(outPath_)
					       + outPrefix_ + "extractedPlane_"
					       + std::to_string(i) +".pcd",
					       *extractedPlane, true);

		// also need to extract inlier indices from the normals, if they
		// were computed, so that the normals and points are consistent
		// with each other.
		if (doComputeNormals_){
		    extractNormal.setIndices(inliers);
		    extractNormal.filter(*normals);
		}

		// Extract non-inliers
		extract.setNegative(true); // Extract the points which are not inliers
		extract.filter(*remainingPoints);
		intermediateCloud.swap(remainingPoints);
		ROS_INFO("Intermediate cloud after swapping in non-inlier points: %d",
			 (int)intermediateCloud->size());
	    }

	    ROS_INFO("All planes size after loop: %d", (int)allPlanes->size());
	    ROS_INFO("Remaining points size after loop: %d",
		     (int)intermediateCloud->size());
	    ROS_INFO("Normals size after loop: %d",
		     (int)normals->size());

	    // Make sure to swap out the fully filtered intermediate cloud into
	    // the cloud that will be visible outside
	    cloud.swap(intermediateCloud);
	    
	    ROS_INFO("Outputting results to: %s", outPath_.c_str());

	    // Write the extracted planes and the remaining points to separate files
	    writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(outPath_)
					   + outPrefix_ + "allPlanes.pcd",
					   *allPlanes, true);
	    writer.write<pcl::PointXYZRGB>(SysUtil::fullDirPath(outPath_)
					   + outPrefix_ + "nonPlanes.pcd",
					   *cloud, true);
	    ROS_INFO("Done");

	    // colour the inliers so we can tell them apart easily
	    for (auto it = allPlanes->begin(); it != allPlanes->end(); it++) {
		it->r = 255;
		it->b = 255;
	    }

	    // After the process is finished, combine the extracted planes and the other
	    // points together again so that they can be displayed
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    *fullCloud = *allPlanes + *remainingPoints;
	}

	/** 
	 * Compute surface normals for each point in the given cloud.
	 * 
	 * @param cloud Cloud from which to compute normals
	 * @param normals This pointcloud will be populated with the normals
	 * @param cloudTransform Use this to set the viewpoint, ensures
	 * normals point in the direction of the viewpoint.
	 * @param radius The radius in which to search for points to use to
	 * compute the normals for a specific point
	 */
	void PreprocessRoom::computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
					    pcl::PointCloud<pcl::Normal>::Ptr& normals,
					    const tf::StampedTransform& cloudTransform)	{
	    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
	    
	    ne.setInputCloud(cloud);
	   	    
	    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	    // set the viewpoint using the origin of the room. This assumes that
	    // the transform has already been applied to the cloud.
	    ne.setViewPoint(cloudTransform.getOrigin().getX(),
			    cloudTransform.getOrigin().getY(),
			    cloudTransform.getOrigin().getZ());
	    ne.setSearchMethod(tree);
	    ne.setRadiusSearch(normalRadius_);
	    ne.compute(*normals);
	}
    } // namespace preprocessing
} // namespace obj_search



int main(int argc, char *argv[]) {
    objsearch::preprocessing::PreprocessRoom rp(argc, argv);
//    rp.doProcessing();
}
