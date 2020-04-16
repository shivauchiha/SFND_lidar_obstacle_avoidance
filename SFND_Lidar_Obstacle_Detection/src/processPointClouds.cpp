// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>) ;
    typename pcl::VoxelGrid<PointT> sor;
     pcl::PointIndices::Ptr roof{new pcl::PointIndices} ;
    sor.setInputCloud(cloud);
    sor.setLeafSize (filterRes,filterRes,filterRes);
    sor.filter(*cloud_filtered);

    // region_crop
    typename pcl::CropBox<PointT> cor;
    cor.setInputCloud(cloud_filtered);
    cor.setMax(maxPoint);
    cor.setMin(minPoint);
    cor.filter(*cloud_filtered);

    // remove_roof
    std::vector<int> indices;
    typename pcl::CropBox<PointT> roo;
    roo.setInputCloud(cloud_filtered);
    roo.setMin(Eigen::Vector4f (-1.5, -1.7,-1, 1));
    roo.setMax(Eigen::Vector4f (2.6,1.7,-0.4, 1));
    roo.filter(indices);

    for(int point : indices)
    roof->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(roof);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);


    

   

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    
  // loop for inliers
    for(int index:inliers->indices)
      planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
  
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> tempinliersResult;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	srand(time(NULL));
	
	// TODO: Fill in this function
    int n_points = cloud->points.size();
	int max_check = 0;
	// For max iterations 
	for (int i = 0;i < maxIterations;i++)
	{
      int pt1 = rand() % n_points;
	  int pt2 = rand() % n_points;
	  int pt3 = rand() % n_points;
	  
	  
	  
	  //finding equation of plane passing through 3 points
	  //pcl::PointXYZ point1 = cloud->points[pt1];
	  //pcl::PointXYZ point2 = cloud->points[pt2];
	  //pcl::PointXYZ point3 = cloud->points[pt3];
    PointT point1 = cloud->points[pt1];
	  PointT point2 = cloud->points[pt2];
	  PointT point3 = cloud->points[pt3];
	  float A = ((point2.y-point1.y)*(point3.z-point1.z))-((point2.z-point1.z)*(point3.y-point1.y));
	  float B = ((point2.z-point1.z)*(point3.x-point1.x))-((point2.x-point1.x)*(point3.z-point1.z));
	  float C = ((point2.x-point1.x)*(point3.y-point1.y))-((point2.y-point1.y)*(point3.x-point1.x));
	  float D = -((A*point1.x)+(B*point1.y)+(C*point1.z));
	  int inlier_count =0;
	  //std::cout<<max_check<<std::endl;
	  for(int i=0; i<n_points;i++ )
	  {
        if((fabs(A*(cloud->points[i].x)+B*(cloud->points[i].y)+C*(cloud->points[i].z)+D)/sqrt(pow(A,2)+pow(B,2)+pow(C,2))) <= distanceTol)
		{
			inlier_count = inlier_count+1;
			tempinliersResult.insert(i);


		}
	  }

	  if(inlier_count>=max_check)
	  {
		  max_check = inlier_count;
		  inliersResult.clear();
		  inliersResult=tempinliersResult;
		  tempinliersResult.clear();

	  }


	}

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
    inliers->indices.insert(inliers->indices.end(),inliersResult.begin(),inliersResult.end());
	// Return indicies of inliers from fitted line with most inliers
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
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    for(pcl::PointIndices getindice : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudcluster (new pcl::PointCloud<PointT>);
        for(int index:getindice.indices)
        
          cloudcluster->points.push_back(cloud->points[index]);
        cloudcluster->width = cloudcluster->points.size();
        cloudcluster->height = 1;
        cloudcluster->is_dense = true;
        clusters.push_back(cloudcluster);
        
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int id,const std::vector<std::vector<float>>& points,std::vector<int>& cluster,KdTree* tree,std::vector<bool>& processed,float distanceTol )
{
  processed[id] = true;
  cluster.push_back(id);
  std::vector<int> nearby = tree->search(points[id],distanceTol);
  for(int id: nearby)
    if(!processed[id])
    {
      proximity (id,points,cluster,tree,processed,distanceTol);
    }
  
}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(),false)  ;
    int id=0;
    while(id<points.size())
     { if(!processed[id])
      {
        std::vector<int> cluster;
        proximity (id,points,cluster,tree,processed,distanceTol);
        clusters.push_back(cluster);
        
      }
	  id++;
	 }
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::custom_Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
       KdTree* tree = new KdTree;
       std::vector<std::vector<float>> c_points;
       for (int i=0; i < cloud->points.size(); i++) 
       {  std::vector<float> c_point{cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
         c_points.push_back(c_point);
         tree->insert(c_point,i);
       }
       std::vector<std::vector<int>> clusters =euclideanCluster(c_points, tree, clusterTolerance);
       //std::cout<<clusters[0].size()<<endl<<clusters[1].size()<<endl;
       
       std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters_ret;
       for(int i = 0;i<clusters.size();i++)
       { typename pcl::PointCloud<PointT>::Ptr cloud_rec {new pcl::PointCloud<PointT> };
        for(int j=0;j<clusters[i].size();j++ )

      {
        if(clusters[i].size()<minSize || clusters[i].size()>maxSize)
        {
          continue;
        }
         cloud_rec->points.push_back(cloud->points[clusters[i][j]]);
       }
         cloud_rec->height=clusters[i].size();
         cloud_rec->width=1;
         cloud->is_dense=true;
         clusters_ret.push_back(cloud_rec);

       }
       return clusters_ret;
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