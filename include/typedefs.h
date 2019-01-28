/* */

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
// #include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>

#include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/3dsc.h>
// #include <pcl/features/usc.h>
#include <pcl/features/shot.h>
// #include <pcl/features/rsd.h>

//SIFT keypoints
#include <pcl/keypoints/sift_keypoint.h>

//Harris keypoints
#include <pcl/keypoints/harris_3d.h>

// registration
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>

//visualizer
// #include <pcl/visualization/cloud_viewer.h>
// #define pause printf("Press Enter key to continue..."); fgetc(stdin);

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;

typedef pcl::Correspondences Correspondences;

typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;

typedef pcl::FPFHSignature33 FPFH_FeatureT;
typedef pcl::VFHSignature308 VFH_FeatureT;
typedef pcl::Histogram<153> SpinImage_FeatureT;
typedef pcl::ShapeContext1980 threeDSC_FeatureT;
// typedef pcl::UniqueShapeContext1960 USC_FeatureT;
typedef pcl::SHOT352 SHOT_FeatureT;
// typedef pcl::PrincipalRadiiRSD RSD_FeatureT;


typedef pcl::PointCloud<pcl::FPFHSignature33> FPFH_Features;
typedef pcl::PointCloud<pcl::VFHSignature308> VFH_Features;
typedef pcl::PointCloud<pcl::Histogram<153>> SpinImage_Features;
typedef pcl::PointCloud<pcl::ShapeContext1980> threeDSC_Features;
// typedef pcl::PointCloud<pcl::UniqueShapeContext1960> USC_Features;
typedef pcl::PointCloud<pcl::SHOT352> SHOT_Features;
// typedef pcl::PointCloud<pcl::PrincipalRadiiRSD> RSD_Features;

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

    // Set point cloud with normal(PointCloudNormal Type).
    void
    setPointCloudNormal (PointCloudNormal::Ptr xyzn)
    {
      xyzn_ = xyzn;
    }

    // Return the pointer to the Point Cloud with Normal 
    // (PointCloudNormal Type).
    PointCloudNormal::Ptr
    getPointCloudNormal () const
    {
      return (xyzn_);
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

    // Set PFH features.
    void setFeatures_PFH (PFH_Features::Ptr pfh_features)
    {
      pfh_features_ = pfh_features;
    }

    // Set VFH features.
    void setFeatures_VFH (VFH_Features::Ptr vfh_features)
    {
      vfh_features_ = vfh_features;
    }

    // Set SpinImage features.
    void setFeatures_SpinImage (SpinImage_Features::Ptr spinimage_features)
    {
      spinimage_features_ = spinimage_features;
    }

    // Set 3DSC features.
    void setFeatures_3DSC (threeDSC_Features::Ptr threedsc_features)
    {
      threedsc_features_ = threedsc_features;
    }

    // // Set USC features.
    // void setFeatures_USC (USC_Features::Ptr usc_features)
    // {
    //   usc_features_ = usc_features;
    // }

    // Set SHOT features.
    void setFeatures_SHOT (SHOT_Features::Ptr shot_features)
    {
      shot_features_ = shot_features;
    }

    // // Set RSD features.
    // void setFeatures_RSD (RSD_Features::Ptr rsd_features)
    // {
    //   rsd_features_ = rsd_features;
    // }

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

    // Get a pointer to the cloud of feature descriptors (VFH).
    VFH_Features::Ptr
    getFeatures_VFH () const
    {
      return (vfh_features_);
    }

    // Get a pointer to the cloud of feature descriptors (SpinImage).
    SpinImage_Features::Ptr
    getFeatures_SpinImage () const
    {
      return (spinimage_features_);
    }

    // Get a pointer to the cloud of feature descriptors (3DSC).
    threeDSC_Features::Ptr
    getFeatures_3DSC () const
    {
      return (threedsc_features_);
    }

    // // Get a pointer to the cloud of feature descriptors (USC).
    // USC_Features::Ptr
    // getFeatures_USC () const
    // {
    //   return (usc_features_);
    // }

    // Get a pointer to the cloud of feature descriptors (SHOT).
    SHOT_Features::Ptr
    getFeatures_SHOT () const
    {
      return (shot_features_);
    }

    // // Get a pointer to the cloud of feature descriptors (RSD).
    // RSD_Features::Ptr
    // getFeatures_RSD () const
    // {
    //   return (rsd_features_);
    // }

  private:
    // Point Cloud Pointer
    PointCloud::Ptr xyz_;

    // Point Cloud Pointer (Key Points)
    PointCloud::Ptr xyz_key_;

    // Transformed Point Cloud (by the Initial Matrix)
    PointCloud::Ptr xyz_transformed_;

    // Key point indices
    pcl::PointIndices::Ptr key_indices_;

    // Point Cloud with Normal
    PointCloudNormal::Ptr xyzn_;

    // Key points with Normal
    PointCloudNormal::Ptr xyzn_key_;

    /*Features*/

      // Normals
    SurfaceNormals::Ptr normals_;

      // Feature: FPFH
    FPFH_Features::Ptr fpfh_features_ ;

      // Feature: PFH
    PFH_Features::Ptr pfh_features_;

      //Feature: VFH
    VFH_Features::Ptr vfh_features_;

      //Feature: SpinImage
    SpinImage_Features::Ptr spinimage_features_;

      //Feature: 3DSC
    threeDSC_Features::Ptr threedsc_features_;

    //   //Feature: USC
    // USC_Features::Ptr usc_features_;

      //Feature: SHOT
    SHOT_Features::Ptr shot_features_;

    //   //Feature: RSD
    // RSD_Features::Ptr rsd_features_;
};

#endif
