/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef CLUSTER3D_H
#define CLUSTER3D_H

#include "../render/render.h"
#include "../render/box.h"
#include <stdint.h>
#include <string>
#include "kdTree3D.h"
#include <pcl/common/common.h>
#include <chrono>
#include <string>


void EuclideanClusterHelper_3D(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree3D* tree, float distanceTol);
std::vector<std::vector<int>> EuclideanCluster_3D(const std::vector<std::vector<float>>& points, KdTree3D* tree, float distanceTol);
                                                      


#endif
