// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
	sor.setLeafSize (filterRes, filterRes, filterRes);    
	sor.filter (*cloudFiltered);


	typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

	pcl::CropBox<PointT> roi(true);
	roi.setMin(minPoint);
	roi.setMax(maxPoint);
	roi.setInputCloud(cloudFiltered);
	roi.filter(*cloudRegion);

	std::vector<int> indices;

	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
	roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
	roof.setInputCloud(cloudRegion);
	roof.filter(indices);

	
	typename pcl::PointCloud<PointT>::Ptr cloudRoofRemoved(new pcl::PointCloud<PointT>);
	// std::cout << "cloudRegion size before roof" << cloudRegion->size() << std::endl;

	for(int i=0; i<(int)cloudRegion->size(); i++) {
		if(find(indices.begin(),indices.end(), i) == indices.end()) cloudRoofRemoved->points.push_back(cloudRegion->points[i]);
	}

	// std::cout << "cloudRegion size after roof" << cloudRoofRemoved->size() << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRoofRemoved;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());

	for (auto i : inliers->indices) {
		planeCloud->points.push_back(cloud->points[i]);
	}

	for(int i=0; i<(int)cloud->size(); i++) {
		if(find(inliers->indices.begin(),inliers->indices.end(), i) == inliers->indices.end()) obstacleCloud->points.push_back(cloud->points[i]);
		else continue;
	}

	// pcl::ExtractIndices<PointT> extract;
	// extract.setInputCloud (cloud);
 //    extract.setIndices (inliers);
 //    extract.setNegative (true);
 //    extract.filter (*obstacleCloud);	
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	
    // TODO:: Fill in this function to find inliers for the cloud.
	// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
 
	// pcl::SACSegmentation<PointT> seg;
	// seg.setOptimizeCoefficients (true);
	// seg.setModelType (pcl::SACMODEL_PLANE);
	// seg.setMethodType (pcl::SAC_RANSAC);
 //  	seg.setMaxIterations (maxIterations);
 //  	seg.setDistanceThreshold (distanceThreshold);


	//     // Segment the largest planar component from the remaining cloud
 //    seg.setInputCloud (cloud);
 //    seg.segment (*inliers, *coefficients);


  	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	while(maxIterations--) {

	// Randomly sample subset and fit line
		int randIndex1 = rand() % cloud->size();
		int randIndex2 = rand() % cloud->size();
		int randIndex3 = rand() % cloud->size();

		auto x1 = cloud->points[randIndex1].x;
		auto y1 = cloud->points[randIndex1].y;
		auto z1 = cloud->points[randIndex1].z;

		auto x2 = cloud->points[randIndex2].x;
		auto y2 = cloud->points[randIndex2].y;
		auto z2 = cloud->points[randIndex2].z;
		
		auto x3 = cloud->points[randIndex3].x;
		auto y3 = cloud->points[randIndex3].y;
		auto z3 = cloud->points[randIndex3].z;

		float A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		float D = -(A*x1 + B*y1 + C*z1);
		// Measure distance between every point and fitted line
		// // If distance is smaller than threshold count it as inlier
		std::unordered_set<int> tempResult;
		for(int index=0; index<(int)cloud->size(); index++) {
			float d = abs(A * cloud->points[index].x + B*cloud->points[index].y + C*cloud->points[index].z + D) / sqrt(A*A + B*B + C*C) ;
			if (d <= distanceThreshold) tempResult.insert(index);
		}
		if(tempResult.size() > inliersResult.size()) inliersResult = tempResult;
	// // Return indicies of inliers from fitted line with most inliers
	}

	inliers->indices.insert(inliers->indices.end(), inliersResult.begin(), inliersResult.end());

	// std::cout<< inliers->indices.size () << "  " << inliersResult.size() << std::endl;
	// std::cout<< inliers->indices.size () << std::endl;
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize (maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	for(auto it : cluster_indices){
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for(auto pit : it.indices){
			cloud_cluster->points.push_back(cloud->points[pit]); }
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
		
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indices, std::vector<bool>& processed, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, std::vector<int>& cluster, float distanceTol) {
	  processed[indices] = true;
	  cluster.push_back(indices);
	  std::vector<float> temp;
	  temp.push_back(cloud->points[indices].x);
	  temp.push_back(cloud->points[indices].y);
	  temp.push_back(cloud->points[indices].z);
	  std::vector<int> nearest = tree->search(temp, distanceTol);
	  for(auto i : nearest) {
	    if(!processed[i]) clusterHelper(i, processed, cloud, tree, cluster, distanceTol);
	  }

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster
	auto startTime = std::chrono::steady_clock::now();

	KdTree* tree = new KdTree();

	for(int i=0; i<cloud->points.size(); i++) {
		std::vector<float> temp;
		temp.push_back(cloud->points[i].x);
		temp.push_back(cloud->points[i].y);
		temp.push_back(cloud->points[i].z);
		tree->insert(temp,i);
	}

	  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	  std::vector<bool> processed(cloud->points.size(), false);

	  for(int i=0; i<cloud->points.size(); i++) {

	    if(!processed[i]){
	      std::vector<int> cluster;
	      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
	      clusterHelper(i, processed, cloud, tree, cluster, distanceTol);

	      if(cluster.size()>=minSize && cluster.size()<=maxSize) {
		    for(int c : cluster) {
		    	cloudCluster->points.push_back(cloud->points[c]);	
		    }
		  	cloudCluster->width = cloudCluster->points.size();
		  	cloudCluster->height = 1;
		  	cloudCluster->is_dense = true;
	  	  	
	      	clusters.push_back(cloudCluster);
	      }
	    }

  }

  	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
	return clusters;

}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}