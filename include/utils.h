#ifndef UTILS_H
#define UTILS_H

#include "typedefs.h"

extern "C" {
	
void filter(const PointCloud::Ptr cloud_src, const PointCloud::Ptr filtered, float &nn_time, float &bd_time);
void downSample(const PointCloud::Ptr cloud_src, const PointCloud::Ptr filtered, float gridsize);
void pointsNumberCheck(FeatureCloud &cloud);

void detectKeyPoints(std::string kernel_keyPoints, const PointCloud::Ptr cloud_ptr, const PointCloud::Ptr keyPoints_cal_ptr);
void describeFeatures(std::string kernel_descriptors, FeatureCloud &cloud, float R, float &nn_time, float &bd_time);

void removeNANFromFPFH(FPFH_Features::Ptr descriptor, FPFH_Features::Ptr nanremoved, FeatureCloud &cloud);

void keyPointsNARF(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_NARF);
void keyPointsSIFT(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_SIFT);
void keyPointsHARRIS(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_Harris);

// for parameters finetune only
void keyPointsSIFTtest(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_SIFT, float min_scale, int n_octaves, int n_scales_per_octave, float min_contrast);
void keyPointsHARRIStest(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_Harris, float radius, float threshold);
// void computeSurfaceNormals (FeatureCloud &cloud, int K);
// void computeFeatures_FPFH (FeatureCloud &cloud, int K);
// void computeFeatures (FeatureCloud &cloud, int normal_K, int FPFH_K);

void computeSurfaceNormals (FeatureCloud &cloud, float R, float &nn_time, float &bd_time);
void computeFeatures_FPFH (FeatureCloud &cloud, float R, float &nn_time, float &bd_time);
void computeFeatures_VFH (FeatureCloud &cloud, float R);
void computeFeatures_SpinImage (FeatureCloud &cloud, float R);
void computeFeatures_3DSC (FeatureCloud &cloud, float R, float &nn_time, float &bd_time);
// void computeFeatures_USC (FeatureCloud &cloud, float R);
void computeFeatures_SHOT (FeatureCloud &cloud, float R, float &nn_time, float &bd_time);
// void computeFeatures_RSD (FeatureCloud &cloud, float R);

void computeFeatures (FeatureCloud &cloud, float normal_R, float FPFH_R);
void construct_PointNormal(FeatureCloud &source_cloud, FeatureCloud &target_cloud, \
  int use_keypoints);

void getIndices (PointCloud::Ptr cloudin, PointCloud::Ptr keypoints_cal, \
  PointCloud::Ptr keypoints_real, pcl::PointIndices::Ptr indices);

void correspondence_estimation(std::string kernel_descriptors, std::string flag_reciprocal, FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &all_corres, float &nn_time, float &bd_time);
void correspondences_rejection(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &correspondences, pcl::Correspondences &inliers, \
  Result *result, int N, float threshold, int use_keypoints);
void iterative_closest_points(std::string solver, std::string flag_reciprocal, std::string flag_ransac, FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  Result *result, float transEps, float corresDist, float EuclFitEps, float outlThresh, \
  int use_keypoints, pcl::Correspondences &inliers, float &nn_time, float &bd_time);

void keyPoint_dist(FeatureCloud &source_cloud, FeatureCloud &target_cloud);

bool string_sort (std::string i,std::string j);
void listdir(std::string path_dir, std::vector<std::string> &files);
void load_bin(std::string infile, FeatureCloud &cloud);

/*
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
visualize_correspondences (const PointCloudPtr points1, const PointCloudPtr keypoints1,
                           const PointCloudPtr points2, const PointCloudPtr keypoints2,
                           Correspondences &correspondences);
*/
}

template <class Feature_Descriptor>
void removeNANFromDescriptor(Feature_Descriptor& feature_descriptor, Feature_Descriptor& nanremoved, FeatureCloud &cloud)
{
    PointCloud::Ptr keyPoints_ptr_nanremoved(new PointCloud);
    pcl::PointIndices::Ptr indices_ptr_nanremoved(new pcl::PointIndices);

    for (int i=0; i<feature_descriptor->points.size();i++)
    {
        //std::cout << feature_descriptor->points[i].histogram[0] << std::endl;
        //if (Feature_Descriptor == "SHOT")
        float p = feature_descriptor->points[i].descriptor[0];
        //if (Feature_Descriptor == "FPFH")
        //	float p = feature_descriptor->points[i].histogram[0];
        if (p != p) 
        {
            continue;
        }
        else
        {
            nanremoved->push_back(feature_descriptor->points[i]);
            keyPoints_ptr_nanremoved->push_back(cloud.getKeyPoints()->points[i]);
            indices_ptr_nanremoved->indices.push_back((cloud.getKeyPoint_indices())->indices[i]);
        }
    }

    if(feature_descriptor->points.size() != nanremoved->points.size())
    {
        cloud.setKeyPoints(keyPoints_ptr_nanremoved);
        cloud.setKeyPoint_indices(indices_ptr_nanremoved);
    }

    std::cout << "Remove NAN From Feature Descriptors:" << std::endl;
    std::cout << (feature_descriptor->points.size() - nanremoved->points.size()) << " Feature Descriptors Are Removed: " \
    << feature_descriptor->points.size() << "->" << nanremoved->points.size() << std::endl << std::endl;
}

#endif

