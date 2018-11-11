/*
  Libraries and Classes.
*/

#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <limits>
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <Eigen/Core>
#include <sys/stat.h>
#include <ctime>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

//filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// key points
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>

#include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

//SIFT keypoints
#include <pcl/keypoints/sift_keypoint.h>

// registration
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>

//visualizer
#include <pcl/visualization/cloud_viewer.h>
#define pause printf("Press Enter key to continue..."); fgetc(stdin);

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

typedef pcl::Correspondences Correspondences;

typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;

typedef pcl::FPFHSignature33 FPFH_FeatureT;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFH_Features;

typedef pcl::search::KdTree<PointT> SearchMethod;

typedef pcl::PFHSignature125 PFH_FeatureT;
typedef pcl::PointCloud<pcl::PFHSignature125> PFH_Features;

// A struct for storing alignment results
struct Result
{
  float fitness_score;
  Eigen::Matrix4f final_transformation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// SIFT
namespace pcl
{
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
        operator () (const PointXYZ & p) const
        {
            return p.z;
        }
    };
}

/*
  Representation of Point Cloud Data
*/
class FeatureCloud
{
  public:
    FeatureCloud () {}

    ~FeatureCloud () {}

    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
    }

    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
    }

    // Return the pointer to the cloud (PointCloud Type).
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Set key point cloud (PointCloud Type).
    void
    setKeyPoints (PointCloud::Ptr xyz_key)
    {
      xyz_key_ = xyz_key;
    }

    // Return the pointer to the key point cloud (PointCloud Type).
    PointCloud::Ptr
    getKeyPoints () const
    {
      return (xyz_key_);
    }

    // Set key point cloud (PointCloud Type).
    void
    setKeyPoint_indices (pcl::PointIndices::Ptr key_indices)
    {
      key_indices_ = key_indices;
    }

    // Return the pointer to the key point cloud (PointCloud Type).
    pcl::PointIndices::Ptr
    getKeyPoint_indices () const
    {
      return (key_indices_);
    }

    // Set initially aligned point cloud (PointCloud Type).
    void
    setTransformedCloud (PointCloud::Ptr xyz_transformed)
    {
      xyz_transformed_ = xyz_transformed;
    }
    // Return the pointer to initially aligned cloud.
    PointCloud::Ptr
    getTransformedCloud () const
    {
      return (xyz_transformed_);
    }

    // Set Normals
    void setSurfaceNormals (SurfaceNormals::Ptr normals) 
    {
      normals_ = normals;
    }
    // Get the pointer to the cloud of 3D surface normals.
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Set FPFH features.
    void setFeatures_FPFH (FPFH_Features::Ptr fpfh_features)
    {
      fpfh_features_ = fpfh_features;
    }

    // Get the pointer to the cloud of feature descriptors (FPFH).
    FPFH_Features::Ptr
    getFeatures_FPFH () const
    {
      return (fpfh_features_);
    }

    // Get a pointer to the cloud of feature descriptors (PFH).
    PFH_Features::Ptr
    getFeatures_PFH () const
    {
      return (pfh_features_);
    }

  private:
    // Point Cloud Pointer
    PointCloud::Ptr xyz_;

    // Point Cloud Pointer (Key Points)
    PointCloud::Ptr xyz_key_;

    // Transformed Point Cloud (Initial Matrix)
    PointCloud::Ptr xyz_transformed_;

    // Key point indices
    pcl::PointIndices::Ptr key_indices_;

    /*Features*/
      // Normals
    SurfaceNormals::Ptr normals_;
      // Feature: FPFH
    FPFH_Features::Ptr fpfh_features_ ;
      // Feature: PFH
    PFH_Features::Ptr pfh_features_;
};

#endif