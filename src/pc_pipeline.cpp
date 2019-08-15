#include "typedefs.h"
#include "utils.h"
#include "modules.h"


int main(int argc, char **argv)
{
	std::string file_name = "config.txt";

	readConfigFile(file_name);
	makeResultFolder(g_RESULT_DIR);

	// ---------------------------------------------------------generate file's datapath
	std::string file_final_pose_matrix = g_RESULT_DIR + "pose_result_kitti.txt";
	std::string file_Ransac_delta_matrix = g_RESULT_DIR + "ransac_delta.txt";
	std::string file_ICP_delta_matrix = g_RESULT_DIR + "icp_delta.txt";
	std::string file_approx_ops_counter = g_RESULT_DIR + "approx_search_ops.txt";
	std::string file_approx_lf_counter = g_RESULT_DIR + "leader_follower_num.txt";

	// ---------------------------------------------------------define variables
	int index_start = 0;
	std::queue<FeatureCloud> clouds;
	std::vector<std::string> path2bins;
	std::vector <int> approx_lf_counter;
	std::vector <int> approx_ops_counter;

	// initialized identity matrix
	Eigen::Matrix4f final_result_kitti = Eigen::Matrix4f::Identity();
	// matrix
	std::ofstream stream_icp_matrix;
	std::ofstream stream_ransac_matrix;
	std::ofstream stream_final_pose_matrix;
	// vector
	std::ofstream stream_approx_lf;
	std::ofstream stream_approx_ops;

	// generate data loading path at path2bins
	listDir(g_DATASET_DIR, path2bins); 
	// Initial matrix for KITTI pose result
	writeMatrix(stream_final_pose_matrix, file_final_pose_matrix, final_result_kitti);

	for (int n = index_start; n < path2bins.size(); n++)
	{
		FeatureCloud cloud_;

		// --------------Stage1: Loading Frame--------------
		loadBINFile(path2bins[n], cloud_);
		// loadTXTFile(path2bins[n], cloud_);

		if (!(n == index_start) && !(n == (index_start + 1)))
			clouds.pop();
		clouds.push(cloud_);

		// --------------Stage2: Preprocessing--------------
		filter(clouds.back());
		// downSample(clouds.back(), 0.1);

		// --------------Stage3: Normal Calculation--------------
		computeSurfaceNormals (clouds.back(), approx_lf_counter, approx_ops_counter);

		// --------------Stage4: Key Points Detection--------------
		detectKeyPoints(clouds.back());

		// --------------Stage5: Feature Description--------------
		describeFeatures(clouds.back());


		if (n == index_start)	// continue to load 2nd frame
		{
			approx_lf_counter.clear();
			approx_ops_counter.clear();
			continue;
		}

		// -------------Stage6: Correspondence Estimation--------------
		Correspondences all_correspondences;
		Correspondences inliers;

		estimateCorrespondence((clouds.back()), (clouds.front()), all_correspondences); // (source, target)

		// ------------Stage7: Corresponence Rejection--------------
		Result* init_result_ptr, init_result;
		init_result_ptr = &init_result;

		rejectCorrespondences((clouds.back()), (clouds.front()), \
		  all_correspondences, inliers, init_result_ptr); // (source, target)

		/** Constructing PointNormal **/
		constructPointNormal(clouds.back(), clouds.front());

		// -----------Stage8: Pose Estimation (ICP)--------------
		Result* icp_result_ptr, icp_result;
		icp_result_ptr = &icp_result;

		iterativeClosestPoints((clouds.back()), (clouds.front()), icp_result_ptr,\
			inliers, approx_lf_counter, approx_ops_counter);

		Eigen::Matrix4f final_result = icp_result_ptr->final_transformation * init_result_ptr->final_transformation;
		final_result_kitti = final_result_kitti * final_result;


		// ---------------------------------------------------------Stage9: Record Results
		printMatrix("ICP", icp_result_ptr->final_transformation);
		printMatrix("Final Pose", final_result_kitti);

		// Leader follower points counter
		writeVector(stream_approx_lf, file_approx_lf_counter, approx_lf_counter);

		// Leader follower operations counter
		writeVector(stream_approx_ops, file_approx_ops_counter, approx_ops_counter);

		/* Ransac delta */
		writeMatrix(stream_ransac_matrix, file_Ransac_delta_matrix, init_result_ptr->final_transformation);

		/* ICP delta */
		writeMatrix(stream_icp_matrix, file_ICP_delta_matrix, icp_result_ptr->final_transformation);

		/* Accumulated result*/
		writeMatrix(stream_final_pose_matrix, file_final_pose_matrix, final_result_kitti);
	}
	return 0;
}
