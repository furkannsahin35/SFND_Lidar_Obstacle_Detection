/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

//#include "../../render/render.h"
//#include <unordered_set>
//#include <pcl/common/common.h>
//#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
//#include "../../processPointClouds.cpp"

#include "ransac3D.h"

template<typename PointT>
std::unordered_set<int> Ransac_3D<PointT>::Ransac3DSegmentation(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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

		while (inliers.size() <= 3)
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
		//		  	(z2-z1)(x3-x1)-(x2-x1)(z3-z1),
		//		  	(x2-x1)(y3-y1)-(y2-y1)(x3-x1) > = <i,j,k> 


		float V3_a = ((V1_b * v2_c) - (V1_c * v2_b));
		float V3_b = ((V1_c * V2_a) - (V1_a * v2_c));
		float V3_c = ((V1_a * v2_b) - (V1_b * V2_a));
		float V4_d = -((x1 * V3_a) + (y1 * V3_b) + (z1 * V3_c));


		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0) // if sampled before contiune
				continue;

			PointT point = cloud->points[index];
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

	std::unordered_set<int> inliers = inliersResult;
	
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Randsac took" << elapsedTime.count() << "milliseconds " << std::endl;
	return inliersResult;

}


