/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> tempinliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    int n_points = cloud->points.size();
	int max_check = 0;
	// For max iterations 
	for (int i = 0;i < maxIterations;i++)
	{
      int start_pt = rand() % n_points;
	  int end_pt = rand() % n_points;
	  
	  
	  
	  //finding equation of line passing through 2 points
	  pcl::PointXYZ point1 = cloud->points[start_pt];
	  pcl::PointXYZ point2 = cloud->points[end_pt];
	  int A = point1.y-point2.y;
	  int B = point2.x-point1.x;
	  int C = (point1.x*point2.y)-(point2.x*point1.y);
	  int inlier_count =0;
	  std::cout<<max_check<<std::endl;
	  for(int i=0; i<n_points;i++ )
	  {
        if((fabs(A*(cloud->points[i].x)+B*(cloud->points[i].y)+C)/sqrt(pow(A,2)+pow(B,2))) <= distanceTol)
		{
			inlier_count = inlier_count+1;
			tempinliersResult.insert(i);


		}
	  }

	  if(inlier_count>max_check)
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

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> tempinliersResult;
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
	  pcl::PointXYZ point1 = cloud->points[pt1];
	  pcl::PointXYZ point2 = cloud->points[pt2];
	  pcl::PointXYZ point3 = cloud->points[pt3];
	  float A = ((point2.y-point1.y)*(point3.z-point1.z))-((point2.z-point1.z)*(point3.y-point1.y));
	  float B = ((point2.z-point1.z)*(point3.x-point1.x))-((point2.x-point1.x)*(point3.z-point1.z));
	  float C = ((point2.x-point1.x)*(point3.y-point1.y))-((point2.y-point1.y)*(point3.x-point1.x));
	  float D = -((A*point1.x)+(B*point1.y)+(C*point1.z));
	  int inlier_count =0;
	  std::cout<<max_check<<std::endl;
	  for(int i=0; i<n_points;i++ )
	  {
        if((fabs(A*(cloud->points[i].x)+B*(cloud->points[i].y)+C*(cloud->points[i].z)+D)/sqrt(pow(A,2)+pow(B,2)+pow(C,2))) <= distanceTol)
		{
			inlier_count = inlier_count+1;
			tempinliersResult.insert(i);


		}
	  }

	  if(inlier_count>max_check)
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

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}









int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.428);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
