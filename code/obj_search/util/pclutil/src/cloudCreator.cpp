#include <random>
#include <cmath>
#include <limits>
#include <string>

#include "sysutil/sysutil.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

void createSphere(const int npoints, const float radius) {

}

void createPlaneSegment(const int npoints, const pcl::Normal& normal){
    
}

void createBox(const int npoints, const pcl::PointXYZ& centre,
	       const pcl::PointXYZ& dimRange, float normalRadius, std::string outputFile,
	       int r = -1, int g = -1, int b = -1){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::default_random_engine generator;
    // one random generator for each dimension, between 0 and the maximum range
    // of that dimension
    std::uniform_real_distribution<float> xdist(-dimRange.x, dimRange.x);
    std::uniform_real_distribution<float> ydist(-dimRange.y, dimRange.y);
    std::uniform_real_distribution<float> zdist(-dimRange.z, dimRange.z);
    std::bernoulli_distribution invert(0.5);
    std::uniform_int_distribution<int> clip(0,2);
    std::uniform_int_distribution<int> rgb(0,255);

    // if RGB values are not given, generate some to use.
    if (r == -1 || g == -1 || b == -1){
	r = rgb(generator);
	g = rgb(generator);
	b = rgb(generator);
    }

    // store the generated values in a vector so that they can be indexed
    std::vector<float> point(3);
    for (int i = 0; i < npoints; i++) {
	// generate a positive value for each axis, within the given range
	point[0] = xdist(generator);
	point[1] = ydist(generator);
	point[2] = zdist(generator);

	// Randomly select one of the dimensions to be clipped to its maximum,
	// ensuring that each point is placed on one of the six planes of the
	// box.
	int clipDim = clip(generator);
	switch (clipDim) {
	case 0:
	    point[0] = dimRange.x;
	    break;
	case 1:
	    point[1] = dimRange.y;
	    break;
	case 2:
	    point[2] = dimRange.z;
	    break;
	}
		
	// Generate an additional three random values to determine whether the
	// sign of the dimensions will be flipped, to ensure that all sides of
	// the box are evenly populated.
	// if (invert(generator)){ point[0] = -point[0]; }
	// if (invert(generator)){ point[1] = -point[1]; }
	// if (invert(generator)){ point[2] = -point[2]; }

	pcl::PointXYZRGB p;
	p.x = point[0];
	p.y = point[1];
	p.z = point[2];
	p.r = r;
	p.g = g;
	p.b = b;
	
	cloud->push_back(p);
    }

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB>(outputFile, *cloud, true);

    // compute normals too
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
	    
    ne.setInputCloud(cloud);
	   	    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    // set the viewpoint to be a little away from one of the corners
    ne.setViewPoint(2, 2, 2);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normalRadius);
    ne.compute(*normals);

    writer.write<pcl::Normal>(std::string(objsearch::sysutil::removeExtension(outputFile, false) + "normals.pcd"), *normals, true);
}

int main(int argc, char *argv[]) {
    // args - number of points, output file name
    createBox(std::stoi(argv[1]), pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(1, 1, 1), std::stof(argv[2]), argv[3]);
    return 0;
}
