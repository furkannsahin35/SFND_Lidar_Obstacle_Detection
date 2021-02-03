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
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}
/*
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	// Time start
	auto startTime = std::chrono::steady_clock::now();


	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	while(maxIterations--)
	{
		//Randomly pick numbers

		std::unordered_set<int>inliers;  //set means avoid  picking same the index/number twice in a row

		while(inliers.size()<2)
			inliers.insert(rand()%(cloud->points.size()));

		float x1,y1,x2,y2;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = (y1-y2);
		float b = (x1-x2);
		float c = (x1*y2 - x2*y1);

		for (int index = 0 ; index < cloud->points.size(); index++)
		{
			if(inliers.count(index)>0) // if sampled before contiune
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;

			float d = fabs((a*x3)+(b*y3)+c)/sqrt((a*a)+(b*b)); // fabs float abs absulate value

			if (d <= distanceTol)
			{
				inliers.insert(index);
			}
		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}

	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
	std::cout<<"Randsac took"<< elapsedTime.count() << "milliseconds "<< std::endl;
	return inliersResult;

}
*/
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	// Time start 
	auto startTime = std::chrono::steady_clock::now();


	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	while (maxIterations--)
	{
		//Randomly pick numbers

		std::unordered_set<int>inliers;  //set means avoid  picking same the index/number twice in a row

		while (inliers.size() < 3)
		{
			inliers.insert(rand() % (cloud->points.size()));
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// Vector V1 and V2 values
		float V1_a = x2 - x1;
		float V1_b = y2 - y1;
		float V1_c = z2 - z1;
		float V2_a = x3 - x1;
		float v2_b = y3 - y1;
		float v2_c = z3 - z1;
		// V1 x V2 to find normal vector.
		// V1xV2 = <(y2-y1)(z3-z1)-(z2-z1)()y3-y1),
		//		  (z2-z1)(x3-x1)-(x2-x1)(z3-z1),
		//		  (x2-x1)(y3-y1)-(y2-y1)(x3-x1) > = <i,j,k> 


		float V3_a = ((V1_b * v2_c) - (V1_c * v2_b));
		float V3_b = ((V1_c * V2_a) - (V1_a * v2_c));
		float V3_c = ((V1_a * v2_b) - (V1_b * V2_a));
		float V4_d = -((x1 * V3_a) + (y1 * V3_b) + (z1 * V3_c));


		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0) // if sampled before contiune
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float d = fabs((V3_a * x4) + (V3_b * y4) + (V3_c * z4) + V4_d) / sqrt((V3_a * V3_a) + (V3_b * V3_b) + (V3_c * V3_c)); // fabs float abs absulate value

			if (d <= distanceTol)
			{
				inliers.insert(index);
			}
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}

	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Randsac took" << elapsedTime.count() << "milliseconds " << std::endl;
	return inliersResult;

}
int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 500, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}

}
