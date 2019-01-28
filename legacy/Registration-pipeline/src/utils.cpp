#include "typedefs.h"
#include "utils.h"

void filter(const PointCloud::Ptr cloud_src, \
  const PointCloud::Ptr filtered)
{
    // std::cout << "Before Filtering:" << cloud_src->size() << std::endl;
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_src);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*filtered);
    // std::cout << "After Filtering:" << filtered->size() << std::endl;
}

void downSample(const PointCloud::Ptr cloud_src, \
  const PointCloud::Ptr filtered, float gridsize)
{
    pcl::VoxelGrid<PointT> grid; //VoxelGrid
    grid.setLeafSize(gridsize, gridsize, gridsize);
    // std::cout << "Before Downsampling:" << cloud_src->size() << std::endl;
    grid.setInputCloud(cloud_src);
    grid.filter(*filtered); 
    // std::cout << "After Downsampling: " << filtered->size() << std::endl;
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
    range_image.createFromPointCloud (point_cloud, angular_resolution, \
      pcl::deg2rad (360.0f), pcl::deg2rad (180.0f), \
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

    PointCloud& keypoints = *keyPoints_NARF;
    keypoints.points.resize (keypoint_indices.points.size ());
    for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    {
        keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
        // std::cout << keypoint_indices.points[i] << ", "<< keypoints.points[i].index << std::endl;
    }
}
 
void keyPointsSIFT(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_SIFT)
{
    const float min_scale = 0.1f; //the standard deviation of the smallest scale in the scale space   //0.005
    const int n_octaves = 12; //the number of octaves (i.e. doublings of scale) to compute         //6
    const int n_scales_per_octave = 6;//the number of scales to compute within each octave          //4
    const float min_contrast = 0.1f;//the minimum contrast required for detection             //0.005

    // Estimate the sift interest points using z values from xyz as the Intensity variants
    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);

    sift.setInputCloud(cloud_src);
    sift.compute(result);
    //std::cout << "Computing the SIFT points takes " << time.toc() / 1000 << "seconds" << std::endl;
    std::cout << "No of SIFT points in the result are " << result.points.size() << std::endl;

    copyPointCloud(result, *keyPoints_SIFT);
    std::cout << "SIFT points in the result are " << keyPoints_SIFT->points.size() << std::endl;
}

// Compute the surface normals
void
computeSurfaceNormals (FeatureCloud &cloud, float R)
{
    SurfaceNormals::Ptr normals_ (new SurfaceNormals);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

    norm_est.setInputCloud (cloud.getPointCloud());
    // norm_est.setIndices(cloud.getKeyPoint_indices());

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    norm_est.setSearchMethod(tree);
    norm_est.setRadiusSearch(R);
    // norm_est.setKSearch(35);
    // norm_est.setKSearch(K);
    norm_est.compute (*normals_);

    cloud.setSurfaceNormals(normals_);
    // std::cout << "Normals:" << *normals_ << std::endl;
}

// void
// computeFeatures_FPFH (FeatureCloud &cloud, int K)
// {

//     FPFH_Features::Ptr fpfh_features_ (new FPFH_Features);
//     FPFH_Features::Ptr fpfh_features_key_ (new FPFH_Features);

//     pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;

//     fpfh_est.setInputCloud (cloud.getPointCloud());
//     fpfh_est.setInputNormals (cloud.getSurfaceNormals());
//     // fpfh_est.setIndices(cloud.getKeyPoint_indices());
//     // fpfh_est.setInputCloud (cloud.getKeyPoints());
//     // fpfh_est.setInputNormals (cloud.getSurfaceNormals());

//     pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//     fpfh_est.setSearchMethod(tree);
//     fpfh_est.setKSearch(K);
//     // fpfh_est.setRadiusSearch(0.05);

//     fpfh_est.compute (*fpfh_features_);

//     // cloud.setFeatures_FPFH(fpfh_features_);
//     // std::cout << "fpfh size: " << fpfh_features_->points.size() << std::endl;

//     int key_index = 0;
//     for (int i = 0; i < (cloud.getKeyPoint_indices())->indices.size(); i++)
//     {
//       key_index = (cloud.getKeyPoint_indices())->indices[i];
//       pcl::FPFHSignature33 descriptor = fpfh_features_->points[key_index];
//       (fpfh_features_key_->points).push_back(descriptor);
//     }

//     // cloud.setFeatures_FPFH(fpfh_features_);

//     cloud.setFeatures_FPFH(fpfh_features_key_);
//     // std::cout << "fpfh size: " << fpfh_features_key_->points.size() << std::endl;
// }

/* New  */
void
computeFeatures_FPFH (FeatureCloud &cloud, float R)
{

    FPFH_Features::Ptr fpfh_features_ (new FPFH_Features);
    // FPFH_Features::Ptr fpfh_features_key_ (new FPFH_Features);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;

    fpfh_est.setSearchSurface (cloud.getPointCloud());
    fpfh_est.setInputNormals (cloud.getSurfaceNormals());
    fpfh_est.setInputCloud (cloud.getKeyPoints());

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(R);
    // fpfh_est.setKSearch(K);

    fpfh_est.compute (*fpfh_features_);

    cloud.setFeatures_FPFH(fpfh_features_);
}

// Compute the surface normals and local features.
void
computeFeatures (FeatureCloud &cloud, float normal_R, float FPFH_R)
{
  computeSurfaceNormals (cloud, normal_R);
  computeFeatures_FPFH (cloud, FPFH_R);
}

void getIndices (PointCloud::Ptr cloudin, PointCloud::Ptr keypoints_cal, \
  PointCloud::Ptr keypoints_real, pcl::PointIndices::Ptr indices)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudin);
    std::vector<float>pointNKNSquareDistance;
    std::vector<int> pointIdxNKNSearch;

    int indice = 0;
    for (size_t i =0; i < keypoints_cal->size();i++)
    {
        kdtree.nearestKSearch(keypoints_cal->points[i],1,pointIdxNKNSearch,pointNKNSquareDistance);

        indice = pointIdxNKNSearch[0];
        indices->indices.push_back(indice);
        keypoints_real->points.push_back(cloudin->points[indice]);
    }
}

void correspondence_estimation(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &all_corres)
{ 
  pcl::registration::CorrespondenceEstimation<FPFH_FeatureT, FPFH_FeatureT> est;
  est.setInputSource (source_cloud.getFeatures_FPFH());
  est.setInputTarget (target_cloud.getFeatures_FPFH());

  est.determineReciprocalCorrespondences (all_corres);
  std::cout << "corr size:" << all_corres.size() << std::endl;
}

void correspondences_rejection(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &correspondences, pcl::Correspondences &inliers, \
  Result *result, int N, float threshold)
{

  pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
  sac.setInputSource(source_cloud.getKeyPoints ());
  sac.setInputTarget(target_cloud.getKeyPoints ());
  // sac.setInputSource(source_cloud.getPointCloud ());
  // sac.setInputTarget(target_cloud.getPointCloud ());

  /* Set the threshold for rejection iteration */
  sac.setInlierThreshold(threshold);
  sac.setMaximumIterations(N);

  sac.getRemainingCorrespondences(correspondences, inliers);
  Eigen::Matrix4f transformation = sac.getBestTransformation();

  result->final_transformation = transformation;

  // std::cout << "ransac:" << std::endl;
  // for(int i = 0; i < 3; i ++)
  // {
  //   for(int j = 0; j < 4; j ++)
  //   {
  //     std::cout << ("%lf", transformation(i,j));
  //     if (j != 4)
  //       std::cout << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << "corr size (after):" << inliers.size() << std::endl;
  PointCloud::Ptr transformed_cloud (new PointCloud);

  //pcl::transformPointCloud (*(source_cloud.getKeyPoints ()), *transformed_cloud, transformation);
  pcl::transformPointCloud (*(source_cloud.getPointCloud ()), *transformed_cloud, transformation);
  source_cloud.setTransformedCloud(transformed_cloud);
}

void iterative_closest_points(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  Result *result, float transEps, float corresDist, float EuclFitEps, float outlThresh)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  PointCloud Final;

  // icp.setInputSource(source_cloud.getTransformedCloud ());  
  // icp.setInputTarget(target_cloud.getKeyPoints ());

  icp.setInputSource(source_cloud.getTransformedCloud ());  
  icp.setInputTarget(target_cloud.getPointCloud ());

  icp.setUseReciprocalCorrespondences(true);
  icp.setMaximumIterations (50);
  icp.setTransformationEpsilon (transEps);
  icp.setMaxCorrespondenceDistance (corresDist);

  icp.setEuclideanFitnessEpsilon (EuclFitEps);
  icp.setRANSACOutlierRejectionThreshold (outlThresh);

  icp.align(Final);

  result->final_transformation = icp.getFinalTransformation();
  result->fitness_score = icp.getFitnessScore();
}

// Utils: helper functions to parse the Kitti odometry data
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
    input.seekg(sizeof(float), ios::cur);    

    points->push_back(point);
  }

  cloud.setInputCloud(points);
  input.close();
}