#ifndef MODULES_H
#define MODULES_H

#include "typedefs.h"

extern "C" {

void filter(FeatureCloud &cloud);
void downSample(FeatureCloud &cloud, float gridsize);

void keyPointsNARF(FeatureCloud &cloud);
void keyPointsSIFT(FeatureCloud &cloud);
void keyPointsHARRIS(FeatureCloud &cloud);

void constructPointNormal(FeatureCloud &source_cloud, FeatureCloud &target_cloud);
void computeSurfaceNormals (FeatureCloud &cloud, std::vector<int> &LF_points_counter, \
	std::vector<int> &LF_operations_counter);

void computeFeatures_FPFH (FeatureCloud &cloud, float R);
void computeFeatures_SHOT (FeatureCloud &cloud, float R);

void estimateCorrespondence(FeatureCloud &source_cloud, FeatureCloud &target_cloud, \
	pcl::Correspondences &all_corres);
void rejectCorrespondences(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &correspondences, pcl::Correspondences &inliers, \
  Result *result);
void iterativeClosestPoints(FeatureCloud &source_cloud, FeatureCloud &target_cloud, Result *result, \
	pcl::Correspondences &inliers, std::vector<int> &LF_points_counter, std::vector<int> &LF_operations_counter);

}

#endif