#include "typedefs.h"
#include "utils.h"

int main(int argc, char **argv)
{
  std::vector<std::string> path2bins;
  std::queue<FeatureCloud> clouds; // FeatureCloud is the class to represent Point Clouds

  // Taking module parameters from the arguments
  std::vector<std::string> arguments(argv, argv + argc);
  // int normal_K = std::stoi(arguments[1]);
  // int FPFH_K = std::stoi(arguments[2]);
  float normal_R = std::stof(arguments[1]);
  float FPFH_R = std::stof(arguments[2]);

  float ransac_threshold = std::stof(arguments[3]); 
  float icp_transEps = std::stof(arguments[4]);
  float icp_corresDist = std::stof(arguments[5]);
  float icp_EuclFitEps = std::stof(arguments[6]);
  float icp_outlThresh = std::stof(arguments[7]);

  std::cout << "normal_R " << normal_R << std::endl;
  std::cout << "FPFH_R " << FPFH_R << std::endl;
  std::cout << "ransac_threshold " << ransac_threshold << std::endl;
  std::cout << "icp_transEps " << icp_transEps << std::endl;
  std::cout << "icp_corresDist " << icp_corresDist << std::endl;
  std::cout << "icp_EuclFitEps " << icp_EuclFitEps << std::endl;
  std::cout << "icp_outlThresh " << icp_outlThresh << std::endl;

  // Path to files where we store the results
  const char * out_file2 = argv[8];
  // const char * dataset_dir = $PATH_TO_DATASET;
  const char * dataset_dir = argv[9];

  int index_start = 0;
  // Initialize the final result matrix
  Eigen::Matrix4f final_result_kitti = Eigen::Matrix4f::Identity();

  // Start loading frame 0
  FeatureCloud cloud_;
  listdir(dataset_dir, path2bins); // the path2bins contains frames (.bin) in a sorted order.
  load_bin(path2bins[index_start], cloud_);
  
  clouds.push(cloud_);

  PointCloud::Ptr cloud_ptr = (clouds.front()).getPointCloud();
  PointCloud::Ptr keyPoints_ptr(new PointCloud);
  PointCloud::Ptr keyPoints_cal_ptr(new PointCloud);
  pcl::PointIndices::Ptr indices_ptr (new pcl::PointIndices);

  /* Modules in the pipeline are defined and implemented externally
    (in a 'plug-and-play' fashion).
  */
  filter(cloud_ptr, cloud_ptr);
  // downSample(cloud_ptr, cloud_ptr, 0.1);
  narfKeyPoints(cloud_ptr, keyPoints_cal_ptr);
  // keyPointsSIFT(cloud_ptr, keyPoints_cal_ptr);
  getIndices(cloud_ptr, keyPoints_cal_ptr, keyPoints_ptr, indices_ptr);
  (clouds.front()).setKeyPoints(keyPoints_ptr);
  (clouds.front()).setKeyPoint_indices(indices_ptr);

  computeFeatures((clouds.front()), normal_R, FPFH_R);

  for (int n=(index_start + 1); n < path2bins.size(); n++)
  {
    // Stage1: Load Point Clouds
    FeatureCloud cloud_;
    load_bin(path2bins[n], cloud_);   
    if (!(n == (index_start + 1)))
      clouds.pop();
    clouds.push(cloud_);

    // Calculate normals and feature descriptors
    PointCloud::Ptr cloud_ptr = (clouds.back()).getPointCloud();
    PointCloud::Ptr keyPoints_cal_ptr(new PointCloud);
    PointCloud::Ptr keyPoints_ptr(new PointCloud);
    pcl::PointIndices::Ptr indices_ (new pcl::PointIndices);

    filter(cloud_ptr, cloud_ptr);
    // downSample(cloud_ptr, cloud_ptr, 0.1);
    narfKeyPoints(cloud_ptr, keyPoints_cal_ptr); // calculate keypoint coordinates
    // keyPointsSIFT(cloud_ptr, keyPoints_cal_ptr);
    getIndices(cloud_ptr, keyPoints_cal_ptr, keyPoints_ptr, indices_); 
    // getIndices: match keypoint coordinates with points in the original point set
    (clouds.back()).setKeyPoints(keyPoints_ptr);
    (clouds.back()).setKeyPoint_indices(indices_);
    computeFeatures(clouds.back(), normal_R, FPFH_R);     // Compute normals and Fpfh

    // Stage3: Coorespondence Estimation
    // (back, front): (source, target)
    Correspondences all_correspondences;
    Correspondences inliers;

    correspondence_estimation((clouds.back()), (clouds.front()), \
    all_correspondences); // (source, target)

    // Stage4: Coarse-alignment Correspondence Rejection
    Result* init_result_ptr, init_result;
    init_result_ptr = &init_result;

    // (source, target) 
    correspondences_rejection((clouds.back()), (clouds.front()), \
      all_correspondences, inliers, init_result_ptr, 1000, ransac_threshold);
    // Stage5: Fine-alignment: Iterative Closest Point 
    Result* icp_result_ptr, icp_result;
    icp_result_ptr = &icp_result;
    
    iterative_closest_points((clouds.back()), (clouds.front()), \
    icp_result_ptr, icp_transEps, icp_corresDist, \
    icp_EuclFitEps, icp_outlThresh);

    // Save results
    Eigen::Matrix4f final_result = icp_result_ptr->final_transformation * \
    init_result_ptr->final_transformation;
    final_result_kitti = final_result_kitti * final_result;

    ofstream myfile_kitti;
    myfile_kitti.open (out_file2, fstream::app);
    // Write the matrix into the pose file.
    for(int i = 0; i < 3; i ++)
    {
      for(int j = 0; j < 4; j ++)
      {
        myfile_kitti << ("%lf", final_result_kitti(i,j));
        if (j != 4)
          myfile_kitti << " ";
      }
    }
    myfile_kitti << "\n";
    myfile_kitti.close();

  }
  return 0;
}