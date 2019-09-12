#include "typedefs.h"
#include "modules.h"
#include "utils.h"

/****

 ****/
void filter(FeatureCloud &cloud)
{
	const PointCloud::Ptr filtered(new PointCloud);
	int num = (cloud.getPointCloud())->size();
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud.getPointCloud());
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*filtered);
	cloud.setInputCloud(filtered);

	std::cout << "Outlier Filtering:" << std::endl;
	std::cout << (num - filtered->size()) << " Points Are Filtered: " \
	<< num << "->" << filtered->size() << std::endl << std::endl;
}

/****
	
 ****/
void downSample(FeatureCloud &cloud, float gridsize)
{
	const PointCloud::Ptr filtered(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	grid.setLeafSize(gridsize, gridsize, gridsize);
	
	std::cout << "Before Downsampling:" << \
	(cloud.getPointCloud())->size() << std::endl;
	
	grid.setInputCloud(cloud.getPointCloud());
	grid.filter(*filtered); 
	cloud.setInputCloud(filtered);
	
	std::cout << "After Downsampling: " << filtered->size() << std::endl;
}

/****
	
 ****/
void keyPointsNARF(FeatureCloud &cloud)
{
	float angular_resolution = 0.5f;
	float support_size = 0.2f;

	const PointCloud::Ptr keyPoints_NARF(new PointCloud);

	PointCloud &point_cloud = *cloud.getPointCloud();
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	pcl::RangeImage::CoordinateFrame coordinate_frame = \
	pcl::RangeImage::CAMERA_FRAME;
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

	angular_resolution = pcl::deg2rad (angular_resolution);
	scene_sensor_pose = Eigen::Affine3f (\
	Eigen::Translation3f (point_cloud.sensor_origin_[0], \
			point_cloud.sensor_origin_[1], \
			point_cloud.sensor_origin_[2])) * \
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

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute (keypoint_indices);

	PointCloud &keypoints = *keyPoints_NARF;
	keypoints.points.resize (keypoint_indices.points.size ());
	
	for (size_t i=0; i<keypoint_indices.points.size (); ++i)
	{
		keypoints.points[i].getVector3fMap () = \
		range_image.points[keypoint_indices.points[i]].getVector3fMap ();
	}
	std::cout << keypoint_indices.points.size () << \
	" NARF Key Points Are Detected." << std::endl << std::endl;

	cloud.setKeyPoints(keyPoints_NARF);
}
 
/****
	
 ****/
void keyPointsSIFT(FeatureCloud &cloud)
{
	//the standard deviation of the smallest scale in the scale space   //0.005
	const float min_scale = 0.03f; 
	//the number of octaves (i.e. doublings of scale) to compute         //6
	const int n_octaves = 8; 
	//the number of scales to compute within each octave          //4
	const int n_scales_per_octave = 12;
	//the minimum contrast required for detection             //0.005
	const float min_contrast = 0.08f;
	
	PointCloud::Ptr keyPoints_SIFT(new PointCloud);

	// Estimate the sift interest points using z values from xyz as the Intensity variants.
	pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);

	sift.setInputCloud(cloud.getPointCloud());
	sift.compute(result);
	copyPointCloud(result, *keyPoints_SIFT);

	cloud.setKeyPoints(keyPoints_SIFT);
	
	std::cout << result.points.size() << \
	" SIFT Key Points Are Detected." << std::endl << std::endl;
}

/****
	
 ****/
void keyPointsHARRIS(FeatureCloud &cloud)
{
	const PointCloud::Ptr cloud_src = cloud.getPointCloud();
	const PointCloud::Ptr keyPoints_Harris_ptr(new PointCloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;

	harris.setInputCloud(cloud_src);

	// Filter only
	harris.setRadius(0.35f);//0.5
	harris.setThreshold(0.01f);//0.02
	harris.setNonMaxSupression(true);
	harris.setRefine(true);

	cloud_out->height = 1;
	cloud_out->width = 100;
	cloud_out->resize(cloud_out->height * cloud_src->width);
	cloud_out->clear();
	harris.compute(*cloud_out);
	int size = cloud_out->size();

	keyPoints_Harris_ptr->height = 1;
	keyPoints_Harris_ptr->width = 100;
	keyPoints_Harris_ptr->resize(cloud_out->height*cloud_src->width);
	keyPoints_Harris_ptr->clear();

	pcl::PointXYZ point;
	for (int i = 0; i<size; i++)
	{
		point.x = cloud_out->at(i).x;
		point.y = cloud_out->at(i).y;
		point.z = cloud_out->at(i).z;
		keyPoints_Harris_ptr->push_back(point);
	}

	cloud.setKeyPoints(keyPoints_Harris_ptr);

	std::cout << keyPoints_Harris_ptr->size() << \
	" HARRIS Key Points Are Detected." << std::endl << std::endl;
}

/****
	
 ****/
void constructPointNormal(FeatureCloud &source_cloud, FeatureCloud &target_cloud)
{
	PointCloudNormal::Ptr pointNormal_src (new PointCloudNormal);
	PointCloudNormal::Ptr pointNormal_tgt (new PointCloudNormal);

	// NOTICE: we need the transformed point cloud here.
	PointCloud::Ptr src_cloud = source_cloud.getTransformedCloud ();
	SurfaceNormals::Ptr src_normal = source_cloud.getSurfaceNormals ();

	for(size_t i = 0; i < src_cloud->points.size(); ++i)
	{
		PointNormal pt_normal;

		pt_normal.x = src_cloud->points[i].x;
		pt_normal.y = src_cloud->points[i].y;
		pt_normal.z = src_cloud->points[i].z;

		pt_normal.normal_x = src_normal->points[i].normal_x;
		pt_normal.normal_y = src_normal->points[i].normal_y;
		pt_normal.normal_z = src_normal->points[i].normal_z;

		pointNormal_src->push_back(pt_normal);
	}
	source_cloud.setPointCloudNormal(pointNormal_src);

	// tgt
	PointCloud::Ptr tgt_cloud = target_cloud.getPointCloud ();
	SurfaceNormals::Ptr tgt_normal = target_cloud.getSurfaceNormals ();

	for(size_t i = 0; i < tgt_cloud->points.size(); ++i)
	{
		PointNormal pt_normal;

		pt_normal.x = tgt_cloud->points[i].x;
		pt_normal.y = tgt_cloud->points[i].y;
		pt_normal.z = tgt_cloud->points[i].z;

		pt_normal.normal_x = tgt_normal->points[i].normal_x;
		pt_normal.normal_y = tgt_normal->points[i].normal_y;
		pt_normal.normal_z = tgt_normal->points[i].normal_z;

		pointNormal_tgt->push_back(pt_normal);
	}
	target_cloud.setPointCloudNormal(pointNormal_tgt);
}

/****
	
 ****/
void computeSurfaceNormals (FeatureCloud &cloud, std::vector<int> &LF_points_counter, \
	std::vector<int> &LF_operations_counter)
{
	SurfaceNormals::Ptr normals_ (new SurfaceNormals);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

	norm_est.setInputCloud (cloud.getPointCloud());
	// norm_est.setIndices(cloud.getKeyPoint_indices());

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	norm_est.setSearchMethod(tree);
	norm_est.setRadiusSearch(g_NORMAL_SEARCH_RADIUS);

	norm_est.setUseCustomizedKDTree(g_NORMAL_USE_CUSTOMIZED_KDTREE);
	norm_est.setMaxLeafSize(g_NORMAL_MAX_LEAF_SIZE);
	norm_est.setApproxRadiusPara(g_APPROX_RADIUS_SEARCH_PARA);

	norm_est.setSaveApproxData(g_SAVE_APPROX_DATA);

	norm_est.compute (*normals_);

	cloud.setSurfaceNormals(normals_);
	removeNANFromNormal(cloud);

	// record leader and follower points number
	LF_points_counter.push_back(norm_est.getApproxLeadersNum());
	LF_points_counter.push_back(norm_est.getApproxFollowersNum());
	LF_points_counter.push_back(norm_est.getApproxLeadersNum() + norm_est.getApproxFollowersNum());
	LF_operations_counter.push_back(norm_est.getApproxRadiusOpsNum());
}

/****
	
 ****/
void computeFeatures_FPFH (FeatureCloud &cloud, float R)
{
	FPFH_Features::Ptr fpfh_features_ (new FPFH_Features);
	FPFH_Features::Ptr nanremoved_(new FPFH_Features);

	pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setSearchSurface (cloud.getPointCloud());
	fpfh_est.setInputNormals (cloud.getSurfaceNormals());
	fpfh_est.setInputCloud (cloud.getKeyPoints());

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	fpfh_est.setSearchMethod(tree);
	fpfh_est.setRadiusSearch(R);

	fpfh_est.compute (*fpfh_features_);

	removeNANFromFPFH(fpfh_features_, nanremoved_, cloud);

	cloud.setFeatures_FPFH(nanremoved_);
}

/****
	
 ****/
 void computeFeatures_SHOT (FeatureCloud &cloud, float R)
 {
	SHOT_Features::Ptr shot_features_(new SHOT_Features);
	SHOT_Features::Ptr nanremoved_(new SHOT_Features);
	pcl::SHOTEstimation<PointT, pcl::Normal, pcl::SHOT352> shot_est;
	shot_est.setSearchSurface(cloud.getPointCloud());
	shot_est.setInputNormals(cloud.getSurfaceNormals());
	shot_est.setInputCloud(cloud.getKeyPoints());
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	shot_est.setSearchMethod(tree);
	shot_est.setRadiusSearch(R); //0.5

	shot_est.compute(*shot_features_);

	removeNANFromDescriptor<SHOT_Features::Ptr>(shot_features_, nanremoved_, cloud);

	cloud.setFeatures_SHOT(nanremoved_);
}

void estimateCorrespondence(FeatureCloud &source_cloud, FeatureCloud &target_cloud, \
	pcl::Correspondences &all_corres)
{ 
	if (g_FEATURE_MODULE == "FPFH")
	{
		pcl::registration::CorrespondenceEstimation<FPFH_FeatureT, FPFH_FeatureT> est;
		est.setInputSource (source_cloud.getFeatures_FPFH());
		est.setInputTarget (target_cloud.getFeatures_FPFH());

		if(g_CORR_EST_USE_RECIPROCAL_SEARCH)
			est.determineReciprocalCorrespondences (all_corres);
	}

	if (g_FEATURE_MODULE == "SHOT")
	{
		pcl::registration::CorrespondenceEstimation<SHOT_FeatureT, SHOT_FeatureT> est;
		est.setInputSource (source_cloud.getFeatures_SHOT());
		est.setInputTarget (target_cloud.getFeatures_SHOT());

		if(g_CORR_EST_USE_RECIPROCAL_SEARCH)
			est.determineReciprocalCorrespondences (all_corres);
	}
}

/****
	
 ****/
void rejectCorrespondences(FeatureCloud &source_cloud, \
	FeatureCloud &target_cloud, pcl::Correspondences &correspondences, \
	pcl::Correspondences &inliers, Result *result)
{

	pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
	sac.setInputSource(source_cloud.getKeyPoints ());
	sac.setInputTarget(target_cloud.getKeyPoints ());

	// sac.setInputSource(source_cloud.getPointCloud ());
	// sac.setInputTarget(target_cloud.getPointCloud ());

	// Set the threshold for rejection iteration
	sac.setInlierThreshold(g_RANSAC_THRESHOLD);
	sac.setMaximumIterations(g_RANSAC_MAX_ITERATION);
	sac.getRemainingCorrespondences(correspondences, inliers);

	Eigen::Matrix4f transformation = sac.getBestTransformation();

	result->final_transformation = transformation;

	printMatrix("RANSAC", transformation);

	PointCloud::Ptr transformed_cloud (new PointCloud);

	pcl::transformPointCloud (*(source_cloud.getPointCloud ()), \
		*transformed_cloud, transformation);
	source_cloud.setTransformedCloud(transformed_cloud);

	std::cout << "Correspondences Rejection:" << std::endl;
	std::cout << (correspondences.size() - inliers.size()) << \
	" Correspondences Are Rejected: " << correspondences.size() << "->" << \
	inliers.size() << std::endl << std::endl;
}

/****
	
 ****/
void iterativeClosestPoints(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  Result *result, pcl::Correspondences &inliers, std::vector<int> &LF_points_counter, std::vector<int> &LF_operations_counter)
{
	PointCloudNormal Final;

	PointCloudNormal::Ptr pointNormal_src = source_cloud.getPointCloudNormal();
	PointCloudNormal::Ptr pointNormal_tgt = target_cloud.getPointCloudNormal();

	pcl::registration::CorrespondenceRejector::Ptr ransac_rej \
	(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointNormal> ()); 

	if(g_ICP_SOLVER == "SVD")
	{
		std::cout << "SVD Solver for ICP Is Running!" << std::endl;
		pcl::IterativeClosestPoint<PointNormal, PointNormal> icp;

		if (g_ICP_USE_RECIPROCAL_SEARCH)
			icp.setUseReciprocalCorrespondences(true);
		if (g_ICP_USE_RANSAC)
		{
			icp.setRANSACOutlierRejectionThreshold (g_ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD);
			icp.addCorrespondenceRejector(ransac_rej);
		}

		icp.setInputSource(pointNormal_src);  
		icp.setInputTarget(pointNormal_tgt);

		icp.setUseCustomizedKDTree(g_ICP_USE_CUSTOMIZED_KDTREE);
		icp.setMaxLeafSize(g_ICP_MAX_LEAF_SIZE);
		icp.setApproxNNPara(g_APPROX_NEAREST_SEARCH_PARA);
		
		icp.setMaximumIterations (g_ICP_MAX_ITERATION);
		icp.setTransformationEpsilon (g_ICP_TRANSFORMATION_EPSILON);
		icp.setMaxCorrespondenceDistance (g_ICP_MAX_CORRESPONDENCE_DISTANCE);
		icp.setEuclideanFitnessEpsilon (g_ICP_EUCLIDEAN_FITNESS_EPSILON);

		icp.setSaveApproxData(g_SAVE_APPROX_DATA);

		icp.align(Final);

		result->final_transformation = icp.getFinalTransformation();
		// result->fitness_score = icp.getFitnessScore();

		std::cout << "ICP Iteration Number: " << icp.getIterNumber() << std::endl;

		LF_points_counter.push_back(icp.getApproxLeadersNum());
		LF_points_counter.push_back(icp.getApproxFollowersNum());
		LF_points_counter.push_back(icp.getApproxLeadersNum() + icp.getApproxFollowersNum());
		LF_operations_counter.push_back(icp.getApproxNNOpsNum());

	}
	if (g_ICP_SOLVER == "LM")
	{
		std::cout << "LM Solver for ICP Is Running!" << std::endl;
		pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> icp_lm;

		if (g_ICP_USE_RECIPROCAL_SEARCH)
			icp_lm.setUseReciprocalCorrespondences(true);
		if (g_ICP_USE_RANSAC)
		{
			icp_lm.setRANSACOutlierRejectionThreshold (g_ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD);
			// add ransac rejector
			icp_lm.addCorrespondenceRejector(ransac_rej);
		}

		icp_lm.setInputSource(pointNormal_src);
		icp_lm.setInputTarget(pointNormal_tgt);

		icp_lm.setMaximumIterations(g_ICP_MAX_ITERATION);
		icp_lm.setTransformationEpsilon(g_ICP_TRANSFORMATION_EPSILON);
		icp_lm.setMaxCorrespondenceDistance(g_ICP_MAX_CORRESPONDENCE_DISTANCE);
		icp_lm.setEuclideanFitnessEpsilon (g_ICP_EUCLIDEAN_FITNESS_EPSILON);

		icp_lm.align(Final);

		result->final_transformation = icp_lm.getFinalTransformation();
		// result->fitness_score = icp_lm.getFitnessScore();
	}
}
