#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>

//filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// key points
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

// registration
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef pcl::Correspondences Correspondences;

typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;

typedef pcl::FPFHSignature33 FPFH_FeatureT;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFH_Features;

typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

typedef pcl::PFHSignature125 PFH_FeatureT;
typedef pcl::PointCloud<pcl::PFHSignature125> PFH_Features;

// A struct for storing alignment results
struct Result
{
  float fitness_score;
  Eigen::Matrix4f final_transformation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*
  Representation of Point Cloud Data
*/
class FeatureCloud
{
  public:
    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    /*
      Initialize the clouds.
      Pass in an existing cloud or Load from designated PCD files.
    */
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


    // Parameters
    float getNormalRadius() const
    {
      return (normal_radius_);
    }

    float getFeatureRadius() const
    {
      return (feature_radius_);
    }

    SearchMethod::Ptr
    getSearchMethod() const
    {
      return (search_method_xyz_);
    }

  private:
    // Point Cloud Pointer
    PointCloud::Ptr xyz_;

    // Point Cloud Pointer (Key Points)
    PointCloud::Ptr xyz_key_;

    // Transformed Point Cloud (Initial Matrix)
    PointCloud::Ptr xyz_transformed_;

    /*Features*/
      // Normals
    SurfaceNormals::Ptr normals_;
      // Feature: FPFH
    FPFH_Features::Ptr fpfh_features_;
      // Feature: PFH
    PFH_Features::Ptr pfh_features_;

    // Search Methods
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};


/* Utils */
void filtering(const PointCloud::Ptr cloud_src, \
  const PointCloud::Ptr filtered)
{
    std::cout << "Before Filtering:" << cloud_src->size() << std::endl;
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_src);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*filtered);
    std::cout << "After Filtering:" << filtered->size() << std::endl;
}

void downSample(const PointCloud::Ptr cloud_src, \
  const PointCloud::Ptr filtered, float gridsize)
{
    pcl::VoxelGrid<PointT> grid; //VoxelGrid
    grid.setLeafSize(gridsize, gridsize, gridsize);
    std::cout << "Before Downsample:" << cloud_src->size() << std::endl;
    grid.setInputCloud(cloud_src);
    grid.filter(*filtered); 
    std::cout << "After Downsample: " << filtered->size() << std::endl;
}

// Compute the surface normals
void
computeSurfaceNormals (FeatureCloud &cloud)
{
    SurfaceNormals::Ptr normals_ (new SurfaceNormals);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

    norm_est.setInputCloud (cloud.getPointCloud());
    norm_est.setSearchMethod (cloud.getSearchMethod());
    norm_est.setRadiusSearch (cloud.getNormalRadius());

    norm_est.compute (*normals_);

    cloud.setSurfaceNormals(normals_);
}

// Compute the local feature descriptors
void
computeFeatures_FPFH (FeatureCloud &cloud)
{
    FPFH_Features::Ptr fpfh_features_ (new FPFH_Features);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;

    fpfh_est.setInputCloud (cloud.getPointCloud());
    fpfh_est.setInputNormals (cloud.getSurfaceNormals());
    
    fpfh_est.setSearchMethod (cloud.getSearchMethod());
    fpfh_est.setRadiusSearch (cloud.getFeatureRadius());
    
    fpfh_est.compute (*fpfh_features_);

    cloud.setFeatures_FPFH(fpfh_features_);
}

// to-do:computeFeatures_PFH

// Compute the surface normals and local features.
void
computeFeatures (FeatureCloud &cloud)
{
  computeSurfaceNormals (cloud);
  computeFeatures_FPFH (cloud);

  // Add more feature computation here.
}

// Narf Key Point Detection
void narfKeyPoints(const PointCloud::Ptr cloud_src, \
  const PointCloud::Ptr keyPoints_NARF)
{
    //Parameters
    float angular_resolution = 0.5f;
    float support_size = 0.2f;

    PointCloud& point_cloud = *cloud_src;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

    angular_resolution = pcl::deg2rad (angular_resolution);
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);

    // -----Create RangeImage from the PointCloud-----
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                      scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;
    //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
    //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);
    std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    PointCloud& keypoints = *keyPoints_NARF;
    keypoints.points.resize (keypoint_indices.points.size ());
    for (size_t i=0; i<keypoint_indices.points.size (); ++i)
        keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
}

void sac_ia(FeatureCloud &source_cloud, FeatureCloud &target_cloud, \
  Result *result, float min_sample_distance_=0.05f, \
  float max_correspondence_distance_=0.01f*0.01f, int nr_iterations_=500)
{
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
  
  sac_ia_.setMinSampleDistance (min_sample_distance_);
  sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
  sac_ia_.setMaximumIterations (nr_iterations_);

  sac_ia_.setInputSource (source_cloud.getPointCloud ());
  
  sac_ia_.setSourceFeatures (source_cloud.getFeatures_FPFH ());

  sac_ia_.setInputTarget (target_cloud.getPointCloud ());
  sac_ia_.setTargetFeatures (target_cloud.getFeatures_FPFH ());

  pcl::PointCloud<pcl::PointXYZ> registration_output;
  sac_ia_.align (registration_output);

  result->fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
  result->final_transformation = sac_ia_.getFinalTransformation ();
}

void correspondence_estimation(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &all_corres)
{
  pcl::registration::CorrespondenceEstimation<FPFH_FeatureT, FPFH_FeatureT> est;

  est.setInputSource (source_cloud.getFeatures_FPFH());
  est.setInputTarget (target_cloud.getFeatures_FPFH());

  est.determineCorrespondences (all_corres);
}

void correspondences_rejection(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &correspondences, pcl::Correspondences &inliers, int N=1000)
{
  pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
  
  sac.setInputSource(source_cloud.getPointCloud ());
  sac.setInputTarget(target_cloud.getPointCloud ());

  // sac.setInlierThreshold(epsilon);
  sac.setMaximumIterations(N);

  sac.getRemainingCorrespondences(correspondences, inliers);
  
  Eigen::Matrix4f transformation = sac.getBestTransformation();
  PointCloud::Ptr transformed_cloud (new PointCloud);
  pcl::transformPointCloud (*source_cloud.getPointCloud (), *transformed_cloud, transformation);

  source_cloud.setTransformedCloud(transformed_cloud);

  std::cout << "Transformation (RanSac): " << std::endl << transformation << std::endl;
}

void iterative_closest_points(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  Result *result)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  PointCloud Final;

  // icp.setInputSource(source_cloud.getPointCloud ());
  icp.setInputSource(source_cloud.getTransformedCloud ());  
  icp.setInputTarget(target_cloud.getPointCloud ());

  icp.setMaximumIterations(100);
  icp.align(Final);

  result->final_transformation = icp.getFinalTransformation();
  result->fitness_score = icp.getFitnessScore();
}

int main(int argc, char **argv)
{

	/*
		Section1: To load a series of PCDs.

		Here in this section, we use another individual txt file to specify point clouds we wanto stitch.
    The content in the .txt file should be like:
    
    ./data/object_template_0.pcd
    ./data/object_template_1.pcd

    And the path to this .txt file should be an argv argument when executing the program.
	*/

  // Point Clouds are represented as instances of the FeatureCloud class.
	std::vector<FeatureCloud> object_templates;

	std::ifstream input_stream (argv[1]);
	object_templates.resize (0);
	std::string pcd_filename;

	while (input_stream.good ())
	{
		std::getline (input_stream, pcd_filename);
		if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
			continue;

		FeatureCloud template_cloud;
		template_cloud.loadInputCloud (pcd_filename);
		object_templates.push_back (template_cloud);

	}
	input_stream.close ();

  std::cout << "Total number of clouds: " << object_templates.size() << std::endl;
  if (object_templates.size() < 2)
  {
    std::cout << "Please specify clouds for registration." << std::endl;
    return -1;
  }  

  // Till now, the point clouds should have been loaded and stored as FeatureCloud instances.

  /*
      Section 2: Preprocessing.

        Filtering, DownSampling, KeyPoint Detection, 
        Feature and Normal calculation, ...
  */

	for (size_t i = 0; i != object_templates.size(); ++i)
	{
		PointCloud::Ptr cloud_ptr = object_templates[i].getPointCloud();
		PointCloud::Ptr keyPoints_ptr(new PointCloud);

    filtering(cloud_ptr, cloud_ptr);
    downSample(cloud_ptr, cloud_ptr, 0.05);
    narfKeyPoints(cloud_ptr, keyPoints_ptr);
    object_templates[i].setKeyPoints(keyPoints_ptr);  

    // Compute normals and features such as pfh.
    computeFeatures(object_templates[i]);
  }

  /*
    Section 3: Alignment.
    
    In the first two stages, we already have loaded and calculated the features of the clouds.
    In this stage, we perform alignment based on these features.
  */
  

  // Method1: SAC-IA based method.

  // As a struct, Result stores the transformation matrix and alignment score. 
  Result* sac_ia_result_ptr, sac_ia_result;
  sac_ia_result_ptr = &sac_ia_result;
  
  sac_ia(object_templates[0], object_templates[1], sac_ia_result_ptr);

  std::cout << "Transformation (SAC-IA): " << std::endl << sac_ia_result.final_transformation << std::endl;
  std::cout << std::endl << "Score: " << sac_ia_result.fitness_score << std::endl <<std::endl;

  //Method2: Correspondence Estimation & Rejection.
  Correspondences all_correspondences;
  Correspondences inliers;

  correspondence_estimation(object_templates[0], object_templates[1], \
    all_correspondences);

  correspondences_rejection(object_templates[0], object_templates[1], \
    all_correspondences, inliers);

  // ICP
  Result* final_result_ptr, final_result;
  final_result_ptr = &final_result;

  iterative_closest_points(object_templates[0], object_templates[1], \
    final_result_ptr);
  
  std::cout << "Transformation from ICP: " << std::endl;
  std::cout << final_result.final_transformation << std::endl;
  std::cout << "Score: " << final_result.fitness_score << std::endl;

	return 0;
}