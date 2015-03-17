/**
 * @file   planeTest.cpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Mon Mar  9 14:06:36 2015
 * 
 * @brief  Some prototype code for doing preprocessing on point clouds.
 * 
 * 
 */
#include "planeTest.hpp"

namespace objsearch {
    namespace preprocessing {

	/** 
	 * Constructor for the preprocessing node class. Uses parameters
	 * received from the parameter server to set up internal attributes.
	 * Once the object has been constructed, it calls functions to process
	 * the given point cloud.
	 * 
	 * @param argc 
	 * @param argv 
	 */
	PreprocessRoom::PreprocessRoom(int argc, char* argv[]){
	    ros::init(argc, argv, "planetest");
	    ros::NodeHandle handle;
	    
	    ROSUtil::getParam(handle, "/planetest/cloud_file", cloudFile);
    
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloudFile, *originalCloud) != -1){
		std::cout << "Loaded cloud from " << cloudFile.c_str() << std::endl;
	    } else {
		std::cout << "Could not load cloud from " << cloudFile.c_str() << std::endl;
		exit(1);
	    }
    
	    ROSUtil::getParam(handle, "/obj_search/raw_data_dir", dataPath);
	    // If the given cloud file corresponds to a file in the raw data directory,
	    // extract the remaining directories in the path of the file so that the
	    // data can be put into the output directory with the same path.
	    if (cloudFile.compare(0, dataPath.size(), dataPath) == 0){
		dataSubDir = SysUtil::trimPath(std::string(cloudFile, dataPath.size()), 1);
	    }

	    ROSUtil::getParam(handle, "/planetest/output_dir", outDir);
	    // If output is not specified, set the output directory to be the processed
	    // data directory specified by the global parameters.
	    if (std::string("NULL").compare(outDir) == 0) {
		ROSUtil::getParam(handle, "/obj_search/processed_data_dir", outDir);
	    }

	    ROSUtil::getParam(handle, "/planetest/RANSAC_distance_threshold", ransacDistanceThresh);
	    ROSUtil::getParam(handle, "/planetest/RANSAC_iterations", ransacIterations);
	    ROSUtil::getParam(handle, "/planetest/planes_to_extract", planesToExtract);

	    extractPlanes(originalCloud);
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
	 */
	void PreprocessRoom::transformAndRemoveFloorCeiling(){}

	/** 
	 * Once the room has been stripped of its floor and ceiling, remove
	 * other planes from the room. We aim to remove walls and other large
	 * flat surfaces from which we do not want to extract features. This
	 * process is done using a basic RANSAC procedure.
	 * 
	 */
	void PreprocessRoom::extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	    seg.setOptimizeCoefficients(true);
	    seg.setModelType(pcl::SACMODEL_PLANE);
	    seg.setMethodType(pcl::SAC_RANSAC);
	    seg.setDistanceThreshold(ransacDistanceThresh);
	    seg.setMaxIterations(ransacIterations);

	    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	    // Points will be removed from this cloud - at the end of the process it
	    // will contain all the points which were not extracted by the segmentation
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intermediateCloud (cloud);
	    // At each stage, the inliers of the plane model will be extracted to this
	    // cloud
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractedPlane (new pcl::PointCloud<pcl::PointXYZRGB>);
	    // All of the points that are extracted throughout the process will end up
	    // in this cloud
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr allPlanes (new pcl::PointCloud<pcl::PointXYZRGB>);
	    // The points which are not inliers to the plane will be placed into this
	    // cloud and then swapped into intermediateCloud
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr remainingPoints (new pcl::PointCloud<pcl::PointXYZRGB>);

	    ROS_INFO("Starting plane extraction.");
	    for (int i = 0; i < planesToExtract; i++) {
		ROS_INFO("Extracting plane %d", i + 1);
		seg.setInputCloud(intermediateCloud);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size () == 0) {
		    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		    break;
		}


		// Extract the inliers
		extract.setInputCloud(intermediateCloud);
		extract.setIndices(inliers);
		extract.setNegative(false); // Extract the points which are inliers
		extract.filter(*extractedPlane);
		*allPlanes += *extractedPlane; // Add the extracted inliers to the cloud of all planes
	
		// Extract non-inliers
		extract.setNegative(true); // Extract the points which are not inliers
		extract.filter(*remainingPoints);
		intermediateCloud.swap(remainingPoints);
	    }

	    // create the directory for output if it has not already been created
	    std::string outPath = SysUtil::combinePaths(outDir, dataSubDir);
	    std::cout << "Outputting results to: " << outPath << std::endl;
	    if (SysUtil::makeDirs(outPath)){
		pcl::PCDWriter writer;
		// Write the extracted planes and the remaining points to separate files
		writer.writeBinary<pcl::PointXYZRGB>(SysUtil::fullDirPath(outPath) + "allPlanes.pcd", *allPlanes);
		writer.writeBinary<pcl::PointXYZRGB>(SysUtil::fullDirPath(outPath) + "nonPlanes.pcd", *remainingPoints);
		std::cout << "Done." << std::endl;
	    } else {
		std::cout << "Could not write point clouds to output directory." << std::endl;
	    }

	    // colour the inliers so we can tell them apart easily
	    for (auto it = allPlanes->begin(); it != allPlanes->end(); it++) {
		it->r = 255;
		it->b = 255;
	    }

	    // After the process is finished, combine the extracted planes and the other
	    // points together again so that they can be displayed
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    *fullCloud = *allPlanes + *remainingPoints;

	    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Cloud viewer"));
	    std::string cloudName("cloud");
	    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(fullCloud);
	    viewer->setBackgroundColor(0,0,0);
	    viewer->addPointCloud(fullCloud, rgb, cloudName.c_str());
	    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudName.c_str());
	    viewer->initCameraParameters();
    
	    while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	    }
	}
    } // namespace preprocessing
} // namespace objsearch

int main(int argc, char *argv[]) {
    objsearch::preprocessing::PreprocessRoom rp(argc, argv);
}
