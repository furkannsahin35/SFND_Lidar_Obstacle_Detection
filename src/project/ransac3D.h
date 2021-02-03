#ifndef RANSAC3D_H
#define RANSAC3D_H

#include <pcl/common/common.h>
#include <unordered_set>
#include <chrono>
//#include <stdint.h>
#include <string>



template<typename PointT>
class Ransac_3D {
public:
    Ransac_3D() {};
    // Deconstructor 
    ~Ransac_3D() {};

	std::unordered_set<int> Ransac3DSegmentation(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
};

#endif