#ifndef UTILS_H
#define UTILS_H

#include "typedefs.h"

extern "C" {
	
void filter(const PointCloud::Ptr cloud_src, const PointCloud::Ptr filtered);
void downSample(const PointCloud::Ptr cloud_src, const PointCloud::Ptr filtered, float gridsize);

void narfKeyPoints(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_NARF);
void keyPointsSIFT(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_SIFT);
// void computeSurfaceNormals (FeatureCloud &cloud, int K);
// void computeFeatures_FPFH (FeatureCloud &cloud, int K);
// void computeFeatures (FeatureCloud &cloud, int normal_K, int FPFH_K);

void computeSurfaceNormals (FeatureCloud &cloud, float R);
void computeFeatures_FPFH (FeatureCloud &cloud, float R);
void computeFeatures (FeatureCloud &cloud, float normal_R, float FPFH_R);

void getIndices (PointCloud::Ptr cloudin, PointCloud::Ptr keypoints_cal, \
  PointCloud::Ptr keypoints_real, pcl::PointIndices::Ptr indices);

void correspondence_estimation(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &all_corres);
void correspondences_rejection(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &correspondences, pcl::Correspondences &inliers, \
  Result *result, int N, float threshold);
void iterative_closest_points(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  Result *result, float transEps, float corresDist, float EuclFitEps, float outlThresh);

void keyPoint_dist(FeatureCloud &source_cloud, FeatureCloud &target_cloud);

bool string_sort (std::string i,std::string j);
void listdir(std::string path_dir, std::vector<std::string> &files);
void load_bin(std::string infile, FeatureCloud &cloud);

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
visualize_correspondences (const PointCloudPtr points1, const PointCloudPtr keypoints1,
                           const PointCloudPtr points2, const PointCloudPtr keypoints2,
                           Correspondences &correspondences);

}
#endif