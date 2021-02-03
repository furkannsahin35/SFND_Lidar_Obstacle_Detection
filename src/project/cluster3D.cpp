/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

//#include "../../render/render.h"
//#include "../../render/box.h"

#include "cluster3D.h"
//#include "kdTree3D.h"


void EuclideanClusterHelper_3D(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree3D* tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for (int id : nearest)
	{
		if (!processed[id])
		{
			EuclideanClusterHelper_3D(id, points, cluster, processed, tree, distanceTol);
		}
	}
}


std::vector<std::vector<int>> EuclideanCluster_3D(const std::vector<std::vector<float>>& points, KdTree3D* tree, float distanceTol)
{
												

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool>processed(points.size(), false);

	int i = 0;
	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}

		std::vector<int>cluster;
		EuclideanClusterHelper_3D(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}


	return clusters;

}


