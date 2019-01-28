#include "typedefs.h"
#include "utils.h"

int main(int argc, char **argv)
{
	std::vector<std::string> arguments(argv, argv + argc);

	// int normal_K = std::stoi(arguments[1]);
	// int FPFH_K = std::stoi(arguments[2]);
	float normal_R = std::stof(arguments[1]);
	float descriptor_R = std::stof(arguments[2]);
	float ransac_threshold = std::stof(arguments[3]);
	float icp_transEps = std::stof(arguments[4]);
	float icp_corresDist = std::stof(arguments[5]);
	float icp_EuclFitEps = std::stof(arguments[6]);
	float icp_outlThresh = std::stof(arguments[7]);
	const char * out_file2 = argv[8];
	const char * stage_time_file = argv[9];
	const char * dataset_dir = argv[10];
	const char * ransac_delta_file = argv[11];
	const char * icp_delta_file = argv[12];
	const char * kdtree_time_file = argv[22];

	int use_keypoints = std::stoi(arguments[13]);
	// 1: keypoints, 0: all points

	std::string kernel_keyPoints = argv[14];
	std::string kernel_descriptors = argv[15];
	std::string kernel_corrEst = argv[16];
	std::string kernel_corrRej = argv[17];
	std::string corrEst_flag_reciprocal = argv[18];
	std::string icp_solver = argv[19];
	std::string icp_flag_ransac = argv[20];
	std::string icp_flag_reciprocal = argv[21];
/*
	std::cout << "normal_R: " << normal_R << std::endl;
	std::cout << "descriptor_R: " << descriptor_R << std::endl;
	std::cout << "ransac_threshold: " << ransac_threshold << std::endl;
	std::cout << "icp_transEps: " << icp_transEps << std::endl;
	std::cout << "icp_corresDist: " << icp_corresDist << std::endl;
	std::cout << "icp_EuclFitEps: " << icp_EuclFitEps << std::endl;
	std::cout << "icp_outlThresh: " << icp_outlThresh << std::endl;
	std::cout << "kernel_keyPoints: " << kernel_keyPoints << std::endl;
	std::cout << "kernel_descriptors: " << kernel_descriptors << std::endl;
	std::cout << "corrEst_flag_reciprocal: " << corrEst_flag_reciprocal << std::endl;
	std::cout << "icp_solver: " << icp_solver << std::endl;
	std::cout << "icp_flag_ransac: " << icp_flag_ransac << std::endl;
	std::cout << "icp_flag_reciprocal: " << icp_flag_reciprocal << std::endl;
	std::cout << std::endl;
	std::cout << "result files:" << std::endl;
	std::cout << out_file2 << std::endl;
	std::cout << ransac_delta_file << std::endl;
	std::cout << icp_delta_file << std::endl;
	std::cout << stage_time_file << std::endl;
	std::cout << kdtree_time_file << std::endl;
	std::cout << std::endl;
*/

	std::vector<std::string> path2bins;
	std::queue<FeatureCloud> clouds;

	std::vector<float> single_stage_time;
	std::vector<float> stage_kdtree_time;

	struct timespec start, finish;
	double elapsed_secs;

	float nn_time, bd_time;
	nn_time = 0.0;
	bd_time = 0.0;

	Eigen::Matrix4f final_result_kitti = Eigen::Matrix4f::Identity();

	int index_start = 0;

	// Start loading frame 0
	FeatureCloud cloud_;
	listdir(dataset_dir, path2bins); 
	// the path2bins contains frames (.bin) in a sorted order.
	load_bin(path2bins[index_start], cloud_);
	clouds.push(cloud_);

	PointCloud::Ptr cloud_ptr = (clouds.front()).getPointCloud();
	PointCloud::Ptr keyPoints_ptr(new PointCloud);
	PointCloud::Ptr keyPoints_cal_ptr(new PointCloud);
	pcl::PointIndices::Ptr indices_ptr (new pcl::PointIndices);

	filter(cloud_ptr, cloud_ptr, nn_time, bd_time);
	// downSample(cloud_ptr, cloud_ptr, 0.1);

	// Normal Calculation
	computeSurfaceNormals (clouds.front(), normal_R, nn_time, bd_time);
	pointsNumberCheck (clouds.front()); // remove NAN

	// Keypoints Detection
	detectKeyPoints(kernel_keyPoints, cloud_ptr, keyPoints_cal_ptr);

	// Calculate the indices of keypoints
	getIndices(cloud_ptr, keyPoints_cal_ptr, keyPoints_ptr, indices_ptr);
	(clouds.front()).setKeyPoints(keyPoints_ptr);
	(clouds.front()).setKeyPoint_indices(indices_ptr);

	// Feature Descriptor Calculation
	describeFeatures(kernel_descriptors, clouds.front(), descriptor_R, nn_time, bd_time);

	std::ofstream myfile_kitti;
	myfile_kitti.open (out_file2, std::fstream::app);
	std::cout << "writing to " << out_file2 << std::endl;

	// write the matrix into the pose file.
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

	for (int n=(index_start + 1); n < path2bins.size(); n++)
	{

		nn_time = 0.0;
		bd_time = 0.0;
		
		FeatureCloud cloud_;

		clock_gettime(CLOCK_MONOTONIC, &start);
		
		load_bin(path2bins[n], cloud_);
		
		clock_gettime(CLOCK_MONOTONIC, &finish);
		elapsed_secs = (finish.tv_sec - start.tv_sec);
		elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
		
		clouds.push(cloud_);
		if (!(n == (index_start + 1)))
			clouds.pop();

		PCL_ERROR("T loading: %f\n\n", elapsed_secs);
		single_stage_time.push_back(elapsed_secs); 
		//0: loading time

		PointCloud::Ptr cloud_ptr = (clouds.back()).getPointCloud();
		PointCloud::Ptr keyPoints_cal_ptr(new PointCloud);
		PointCloud::Ptr keyPoints_ptr(new PointCloud);
		pcl::PointIndices::Ptr indices_ (new pcl::PointIndices);

		clock_gettime(CLOCK_MONOTONIC, &start);
		
		filter(cloud_ptr, cloud_ptr, nn_time, bd_time);
		
		clock_gettime(CLOCK_MONOTONIC, &finish);
		elapsed_secs = (finish.tv_sec - start.tv_sec);
		elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

		PCL_ERROR("T Filter: %f\n\n", elapsed_secs);
		PCL_ERROR("Search: %f\n", nn_time);
		PCL_ERROR("Build: %f\n", bd_time);
		stage_kdtree_time.push_back(nn_time);
		stage_kdtree_time.push_back(bd_time);
		single_stage_time.push_back(elapsed_secs);
		// 1: filter time
		nn_time = 0.0;
		bd_time = 0.0;
		std::cout << std::endl;

		// Normal
		clock_gettime(CLOCK_MONOTONIC, &start);
		
		computeSurfaceNormals (clouds.back(), normal_R, nn_time, bd_time);
		
		clock_gettime(CLOCK_MONOTONIC, &finish);
		elapsed_secs = (finish.tv_sec - start.tv_sec);
		elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

		PCL_ERROR("T Normal: %f\n\n", elapsed_secs);
		PCL_ERROR("Search: %f\n", nn_time);
		PCL_ERROR("Build: %f\n", bd_time);
		stage_kdtree_time.push_back(nn_time);
		stage_kdtree_time.push_back(bd_time);
		single_stage_time.push_back(elapsed_secs); 
		// 2: Normal calc time
		nn_time = 0.0;
		bd_time = 0.0;
		std::cout << std::endl;

		pointsNumberCheck (clouds.back());

		// Keypoints Detection
		clock_gettime(CLOCK_MONOTONIC, &start);
		
		detectKeyPoints(kernel_keyPoints, cloud_ptr, keyPoints_cal_ptr);

		clock_gettime(CLOCK_MONOTONIC, &finish);
		elapsed_secs = (finish.tv_sec - start.tv_sec);
		elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
		PCL_ERROR("T Keypoint Detection: %f\n\n", elapsed_secs);
		single_stage_time.push_back(elapsed_secs); 
		// #: Keypoint detection time

		getIndices(cloud_ptr, keyPoints_cal_ptr, keyPoints_ptr, indices_);
		(clouds.back()).setKeyPoints(keyPoints_ptr);
		(clouds.back()).setKeyPoint_indices(indices_);

		// Feature Descriptor 
		clock_gettime(CLOCK_MONOTONIC, &start);
		
		describeFeatures(kernel_descriptors, clouds.back(), descriptor_R, \
			nn_time, bd_time);
		
		clock_gettime(CLOCK_MONOTONIC, &finish);
		elapsed_secs = (finish.tv_sec - start.tv_sec);
		elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
		PCL_ERROR("T Feature Descriptor: %f\n", elapsed_secs);
		PCL_ERROR("Search: %f\n", nn_time);
		PCL_ERROR("Build: %f\n", bd_time);
		stage_kdtree_time.push_back(nn_time);
		stage_kdtree_time.push_back(bd_time);
		single_stage_time.push_back(elapsed_secs); 
		// 4: Feature Descriptor Calc time
		nn_time = 0.0;
		bd_time = 0.0;
		std::cout << std::endl;

		// Correspondence Estimation
		Correspondences all_correspondences;
		Correspondences inliers;

		clock_gettime(CLOCK_MONOTONIC, &start);

		// (back, front): (source, target)
		correspondence_estimation(kernel_descriptors, corrEst_flag_reciprocal,\
		(clouds.back()), (clouds.front()), all_correspondences, nn_time, bd_time); 
		
		clock_gettime(CLOCK_MONOTONIC, &finish);
		elapsed_secs = (finish.tv_sec - start.tv_sec);
		elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
		PCL_ERROR("T Correspondence Est: %f\n", elapsed_secs);
		PCL_ERROR("Search: %f\n", nn_time);
		PCL_ERROR("Build: %f\n", bd_time);
		stage_kdtree_time.push_back(nn_time);
		stage_kdtree_time.push_back(bd_time);
		single_stage_time.push_back(elapsed_secs);
		// 5: Correspondence Est time
		nn_time = 0.0;
		bd_time = 0.0;
		std::cout << std::endl;

		//  Corres Rejection
		Result* init_result_ptr, init_result;
		init_result_ptr = &init_result;

		clock_gettime(CLOCK_MONOTONIC, &start);
		correspondences_rejection((clouds.back()), (clouds.front()), \
		  all_correspondences, inliers, init_result_ptr, 1000, \
		  ransac_threshold, use_keypoints); // (source, target)
		
		clock_gettime(CLOCK_MONOTONIC, &finish);
		elapsed_secs = (finish.tv_sec - start.tv_sec);
		elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

		PCL_ERROR("T Coores Rej: %f\n\n", elapsed_secs);
		single_stage_time.push_back(elapsed_secs); 
		// 6: Corres Rej time

		/* Constructing PointNormal type */
		// Here the time consumption is not counted
		construct_PointNormal(clouds.back(), clouds.front(), use_keypoints);

		// ICP
		Result* icp_result_ptr, icp_result;
		icp_result_ptr = &icp_result;

		clock_gettime(CLOCK_MONOTONIC, &start);

		iterative_closest_points(icp_solver, icp_flag_reciprocal, icp_flag_ransac, (clouds.back()), (clouds.front()), \
		icp_result_ptr, icp_transEps, icp_corresDist, \
		icp_EuclFitEps, icp_outlThresh, use_keypoints, inliers, \
		nn_time, bd_time);

		clock_gettime(CLOCK_MONOTONIC, &finish);
		elapsed_secs = (finish.tv_sec - start.tv_sec);
		elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
		PCL_ERROR("T ICP: %f\n\n", elapsed_secs);
		PCL_ERROR("Search: %f\n", nn_time);
		PCL_ERROR("Build: %f\n", bd_time);
		stage_kdtree_time.push_back(nn_time);
		stage_kdtree_time.push_back(bd_time);
		single_stage_time.push_back(elapsed_secs); 
		// 7: ICP time
		nn_time = 0.0;
		bd_time = 0.0;
		std::cout << std::endl;

		/* Dump the results / profiling data */

		Eigen::Matrix4f final_result = icp_result_ptr->final_transformation * \
		init_result_ptr->final_transformation;

		final_result_kitti = final_result_kitti * final_result;

		// save stage time records
		std::ofstream timefile;
		timefile.open (stage_time_file, std::fstream::app);

		for(int i = 0; i < single_stage_time.size(); i ++)
		{
			timefile << ("%lf", single_stage_time[i]);
			if (i != 7)
				timefile << " ";
		}
		timefile << "\n";
		timefile.close();
		single_stage_time.clear();

		// kdtree time (stages)
		std::ofstream kdtree_time;
		kdtree_time.open(kdtree_time_file, std::fstream::app);
		for(int i = 0; i < stage_kdtree_time.size(); i ++)
		{
			kdtree_time << ("%lf", stage_kdtree_time[i]);
			kdtree_time << " ";
		}
		kdtree_time << "\n";
		kdtree_time.close();
		stage_kdtree_time.clear();

		// Results
		std::cout << "ICP:" << std::endl;
		for(int i = 0; i < 3; i ++)
		{
			for(int j = 0; j < 4; j ++)
			{
				std::cout << ("%lf", icp_result_ptr->final_transformation(i,j));
				if (j != 4)
					std::cout << " ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;

		std::cout << "Final:" << std::endl;
		for(int i = 0; i < 3; i ++)
		{
			for(int j = 0; j < 4; j ++)
			{
				std::cout << ("%lf", final_result(i,j));
				if (j != 4)
					std::cout << " ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;

		/* ransac delta */
		std::ofstream myfile_ransac;
		myfile_ransac.open (ransac_delta_file, std::fstream::app);

		std::cout << "writing to " << ransac_delta_file << std::endl;

		for(int i = 0; i < 3; i ++)
		{
			for(int j = 0; j < 4; j ++)
			{
				myfile_ransac << ("%lf", init_result_ptr->final_transformation(i,j));
				if (j != 4)
					myfile_ransac << " ";
			}
		}
		myfile_ransac << "\n";
		myfile_ransac.close();

		/* ransac delta */
		std::ofstream myfile_icp;
		myfile_icp.open (icp_delta_file, std::fstream::app);
		std::cout << "writing to " << icp_delta_file << std::endl;

		for(int i = 0; i < 3; i ++)
		{
			for(int j = 0; j < 4; j ++)
			{
				myfile_icp << ("%lf", icp_result_ptr->final_transformation(i,j));
				if (j != 4)
					myfile_icp << " ";
			}
		}
		myfile_icp << "\n";
		myfile_icp.close();


		/* Accumulated result*/
		std::ofstream myfile_kitti;
		myfile_kitti.open (out_file2, std::fstream::app);
		std::cout << "writing to " << out_file2 << std::endl;

		// write the matrix into the pose file.
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