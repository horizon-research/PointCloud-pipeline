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
#include <dirent.h> 

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

// Filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Key-point modules
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>

#include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>

// Registration modules
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

/** global variable definition**/
extern std::string g_DATASET_DIR;	// Dataset directory
extern std::string g_RESULT_DIR;	// Result directory

// Normal Estimation Parameters
extern float g_NORMAL_SEARCH_RADIUS;
extern bool g_NORMAL_USE_CUSTOMIZED_KDTREE;
extern int g_NORMAL_MAX_LEAF_SIZE;

// Key Point Detection Parameters
extern std::string g_KEYPOINTS_MODULE;

// Feature Description Parameters
extern float g_FEATURE_SEARCH_RADIUS;
extern std::string g_FEATURE_MODULE;

// Correspondence Estimation Parameters
extern bool g_CORR_EST_USE_RECIPROCAL_SEARCH;

// Correspondence Rejection Parameters
extern float g_RANSAC_THRESHOLD;
extern int g_RANSAC_MAX_ITERATION;

// ICP Parameters
extern std::string g_ICP_SOLVER;
extern int g_ICP_MAX_ITERATION;
extern bool g_ICP_USE_RANSAC;
extern bool g_ICP_USE_RECIPROCAL_SEARCH;

extern float g_ICP_TRANSFORMATION_EPSILON;
extern float g_ICP_MAX_CORRESPONDENCE_DISTANCE;
extern float g_ICP_EUCLIDEAN_FITNESS_EPSILON;
extern float g_ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD;

extern bool g_ICP_USE_CUSTOMIZED_KDTREE;
extern int g_ICP_MAX_LEAF_SIZE;

extern float g_APPROX_RADIUS_SEARCH_PARA;
extern float g_APPROX_NEAREST_SEARCH_PARA;

extern bool g_SAVE_APPROX_DATA;


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;

typedef pcl::Correspondences Correspondences;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;

typedef pcl::FPFHSignature33 FPFH_FeatureT;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFH_Features;

typedef pcl::SHOT352 SHOT_FeatureT;
typedef pcl::PointCloud<pcl::SHOT352> SHOT_Features;

typedef pcl::search::KdTree<PointT> SearchMethod;

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

/*************************************
  Customized Point Cloud Data Structure
*************************************/
class FeatureCloud
{
  public:
	FeatureCloud () {}

	~FeatureCloud () {}

	// Sets the pointer to the point cloud.
	void setInputCloud (PointCloud::Ptr xyz)
	{
	  xyz_ = xyz;
	}

	// Loads Point Cloud from a PCD file.
	void loadInputCloud (const std::string &pcd_file)
	{
	  xyz_ = PointCloud::Ptr (new PointCloud);
	  pcl::io::loadPCDFile (pcd_file, *xyz_);
	}

	// Returns the pointer to the cloud (PointCloud Type).
	PointCloud::Ptr getPointCloud () const
	{
	  return (xyz_);
	}

	// Sets key point cloud (PointCloud Type).
	void setKeyPoints (PointCloud::Ptr xyz_key)
	{
	  xyz_key_ = xyz_key;
	}

	// Returns the pointer to the key point cloud (PointCloud Type).
	PointCloud::Ptr getKeyPoints () const
	{
	  return (xyz_key_);
	}

	// Sets the pointer to the PointCloudNormal Type.
	void setPointCloudNormal (PointCloudNormal::Ptr xyzn)
	{
	  xyzn_ = xyzn;
	}

	// Returns the pointer to the PointCloudNormal Type
	PointCloudNormal::Ptr getPointCloudNormal () const
	{
	  return (xyzn_);
	}

	// Sets the pointer to the key-point cloud (PointCloud Type).
	void setKeyPoint_indices (pcl::PointIndices::Ptr key_indices)
	{
	  key_indices_ = key_indices;
	}

	// Returns the pointer to the key-point cloud (PointCloud Type).
	pcl::PointIndices::Ptr getKeyPoint_indices () const
	{
	  return (key_indices_);
	}

	// Sets the pointer to the point cloud transformed by the initial
	// tranformation matrix (PointCloud Type).
	void setTransformedCloud (PointCloud::Ptr xyz_transformed)
	{
	  xyz_transformed_ = xyz_transformed;
	}

	// Returns the pointer to the point cloud transformed by the initial
	// tranformation matrix (PointCloud Type).
	PointCloud::Ptr getTransformedCloud () const
	{
	  return (xyz_transformed_);
	}

	// Sets pointer to Normals.
	void setSurfaceNormals (SurfaceNormals::Ptr normals) 
	{
	  normals_ = normals;
	}
	// Gets the pointer to the cloud of 3D surface normals.
	SurfaceNormals::Ptr getSurfaceNormals () const
	{
	  return (normals_);
	}

	// Sets a pointer to the feature descriptor data (FPFH type).
	void setFeatures_FPFH (FPFH_Features::Ptr fpfh_features)
	{
	  fpfh_features_ = fpfh_features;
	}

	// Sets a pointer to the feature descriptor data (SHOT type).
	void setFeatures_SHOT (SHOT_Features::Ptr shot_features)
	{
	   shot_features_ = shot_features;
	}

	// Gets the pointer to the feature descriptor data (FPFH type).
	FPFH_Features::Ptr getFeatures_FPFH () const
	{
	  return (fpfh_features_);
	}

	// Gets a pointer to the feature descriptor data (SHOT type).
	SHOT_Features::Ptr getFeatures_SHOT () const
	{
	  return (shot_features_);
	}

  private:
	// Pointer to the Original Point Cloud
	PointCloud::Ptr xyz_;

	// Pointer to the Key points
	PointCloud::Ptr xyz_key_;

	// Pointer to the Point Cloud transformed by the Initial Matrix
	PointCloud::Ptr xyz_transformed_;

	// PCL data structure: point indices (for key points)
	pcl::PointIndices::Ptr key_indices_;

	// PCL data structure: Point Cloud with Normal
	PointCloudNormal::Ptr xyzn_;

	// PCL data structure: Key points with Normal
	PointCloudNormal::Ptr xyzn_key_;

	/* Features */

	// Surface Normals
	SurfaceNormals::Ptr normals_;

	// Feature Descriptor: FPFH
	FPFH_Features::Ptr fpfh_features_ ;

	// Feature Descriptor: SHOT
	SHOT_Features::Ptr shot_features_;

	// Add more feature descriptors if needed
};

#endif