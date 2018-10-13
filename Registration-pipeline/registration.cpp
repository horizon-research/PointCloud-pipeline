/* 20181013 pairwise registration pipeline */
#include <limits>
#include <fstream>
#include <iostream>
#include <stdio.h>
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

#include <ctime>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::Correspondences Correspondences;

typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

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
    FPFH_Features::Ptr fpfh_features_ ;
      // Feature: PFH
    PFH_Features::Ptr pfh_features_;

    // Search Methods
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

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

  source_cloud.getFeatures_FPFH();

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

  // std::cout << "Transformation (RanSac): " << std::endl << transformation << std::endl;
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

// utils
bool string_sort (std::string i,std::string j) 
{
  // the part before '.bin' 
  std::string foo_i = i.substr(0,i.length()-4);
  std::string foo_j = j.substr(0,i.length()-4);

  return (foo_i<foo_j); 
}


void listdir(std::string path_dir, std::vector<std::string> &files)
{
  DIR* dirp = opendir(path_dir.c_str());

  int file_num = 0;
  if (!dirp)
  {
    std::cout << path_dir << endl;
    return;
  }

  struct dirent * dp;
  while ((dp = readdir(dirp)) != NULL) {

    std::string str_d_name(dp->d_name);
    
    if (str_d_name == ".")
      continue;
    if (str_d_name == "..")
      continue;
    file_num += 1;

    if (*(path_dir.end() - 1) == '/')
      files.push_back(path_dir + str_d_name);
    else
      files.push_back(path_dir + '/' + str_d_name);

  }
  closedir(dirp);

  std::cout << "total number of frames: " << file_num << std::endl;

  // sort
  std::sort (files.begin(), files.end(), string_sort);

}

void load_bin(std::string infile, FeatureCloud &cloud)
{
  std::cout << "Loading " << infile << std::endl;
  
  fstream input(infile.c_str(), ios::in | ios::binary);
  
  if(!input.good()){
    cerr << "Could not read file: " << infile << endl;
    exit(EXIT_FAILURE);
  }
  input.seekg(0, ios::beg);

  /* convertion */
  PointCloud::Ptr points (new PointCloud);

  for (int j=0; input.good() && !input.eof(); j++) {
    //PointXYZI point;
    PointT point;

    input.read((char *) &point.x, 3*sizeof(float));
    // input.read((char *) &point.intensity, sizeof(float));
    points->push_back(point);
  }

  cloud.setInputCloud(points);
  input.close();
}

int main(int argc, char **argv)
{
	/*
		Section1: To load a series of Point Clouds.
  */
  // Point Clouds are represented as instances of the FeatureCloud class.
	
  if (argc < 2)
  {
    std::cout << "Please specify path to the (sequence) dataset." << std::endl;
    return -1;
  }

  const char * out_file;
  if (argc < 3)
    out_file = "pose_result.txt";
  else
    out_file = argv[2];

  std::vector<std::string> path2bins;
  std::vector<FeatureCloud> clouds;
  clouds.resize (0);

  std::string dataset_dir; // path to the dataset directory
  dataset_dir = argv[1];

  clock_t begin, end;
  double elapsed_secs;

  listdir(dataset_dir, path2bins); // the path2bins contains frames (.bin) in a sorted order.

  // Load frame 0 (the source).
  FeatureCloud cloud_;
  load_bin(path2bins[0], cloud_);
  std::cout << (*cloud_.getPointCloud()).width << ", " <<  
  (*cloud_.getPointCloud()).height << std::endl;

  clouds.push_back(cloud_);

  //Compute Features of frame 0
  PointCloud::Ptr cloud_ptr = clouds[0].getPointCloud();
  PointCloud::Ptr keyPoints_ptr(new PointCloud);

  filtering(cloud_ptr, cloud_ptr);
  downSample(cloud_ptr, cloud_ptr, 0.5);
  narfKeyPoints(cloud_ptr, keyPoints_ptr);
  clouds[0].setKeyPoints(keyPoints_ptr);
  // Compute normals and features such as pfh.
  computeFeatures(clouds[0]);

  // for (int n=1; n < 10; n++)
  for (int n=1; n < dataset_dir.size(); n++)
  {

    begin = clock();
    FeatureCloud cloud_;
    
    // Stage1: Load current frame
    load_bin(path2bins[n], cloud_);   
    std::cout << (*cloud_.getPointCloud()).width << ", " << \
    (*cloud_.getPointCloud()).height << std::endl;
    clouds.push_back(cloud_);
  
    end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time spent on loading: " << elapsed_secs << std::endl;

    // -----------------
    // Stage2: Preprocess current frame
    begin = clock();
    PointCloud::Ptr cloud_ptr = clouds[n].getPointCloud();
    PointCloud::Ptr keyPoints_ptr(new PointCloud);

    filtering(cloud_ptr, cloud_ptr);
    downSample(cloud_ptr, cloud_ptr, 0.5);
    narfKeyPoints(cloud_ptr, keyPoints_ptr);
    clouds[n].setKeyPoints(keyPoints_ptr);
    // Compute normals and features such as pfh.
    computeFeatures(clouds[n]);
    end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time spent on preprocessing " << elapsed_secs << std::endl;

    // SAC-IA method
    // Result* sac_ia_result_ptr, sac_ia_result;
    // sac_ia_result_ptr = &sac_ia_result;
    // sac_ia(clouds[n], clouds[0], sac_ia_result_ptr);
    // std::cout << "Transformation (SAC-IA): " << std::endl << sac_ia_result.final_transformation << std::endl;
    // std::cout << std::endl << "Score: " << sac_ia_result.fitness_score << std::endl <<std::endl;

    // Stage3: Alignment
    begin = clock();

    Correspondences all_correspondences;
    Correspondences inliers;
    
    begin = clock();
    correspondence_estimation(clouds[n], clouds[0], all_correspondences);
    end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time spent on Coorespondence Est: " << elapsed_secs << std::endl;

    begin = clock();
    correspondences_rejection(clouds[n], clouds[0], \
      all_correspondences, inliers);
    end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time spent on Coorespondence Rej: " << elapsed_secs << std::endl;

    // ICP
    Result* final_result_ptr, final_result;
    final_result_ptr = &final_result;
    
    begin = clock();
    iterative_closest_points(clouds[n], clouds[0], \
      final_result_ptr);
    end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time spent on ICP: " << elapsed_secs << std::endl;

    // std::cout << "Transformation from ICP: " << std::endl;
    // std::cout << final_result.final_transformation << std::endl;
    // std::cout << "Score: " << final_result.fitness_score << std::endl;

    end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time spent on Corr Est + Rej + ICP: " << elapsed_secs << std::endl;

    ofstream myfile;
    myfile.open (out_file, fstream::app);
    std::cout << "writing to " << out_file << std::endl;
    // write the matrix into the pose file.
    for(int i = 0; i < 3; i ++)
    {
      for(int j = 0; j < 4; j ++)
      {
        myfile << ("%lf", final_result.final_transformation(i,j));
        if (j != 4)
          myfile << " ";
      }
    }
    myfile << "\n";
    myfile.close();
  }

  return 0;
}
