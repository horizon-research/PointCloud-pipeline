#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

void generate_cloud(const PointCloud::Ptr cloud_in, 
	int width=100, int height=1, bool is_dense=true, bool display=true)
{
	/*
		Randomly generating 3d point cloud data
	*/
	
	srand(time(NULL));

	cloud_in->width = width;
	cloud_in->height = height;
	cloud_in->is_dense = is_dense;
	cloud_in->points.resize (cloud_in->width * cloud_in->height);

	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
		cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

		if (display)
		{
			std::cout << cloud_in->points[i].x << 
			cloud_in->points[i].y <<
			cloud_in->points[i].z << std::endl;
		}
	}

}

void transform_cloud(const PointCloud::Ptr cloud_in)
{
	/* define rigid transformation here */
	float theta = M_PI/4;
	
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	transform.translation() << 3.0, .0, .0;
	transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

	std::cout << "Regid Transformation Ground Truth: " << std::endl << transform.matrix() << std::endl << std::endl;
	pcl::transformPointCloud(*cloud_in, *cloud_in, transform);
}

void vis_clouds2(const PointCloud::Ptr cloud, const PointCloud::Ptr cloud_trans)
{
	pcl::visualization::PCLVisualizer viewer ("Matrix transformation");

	// Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler (cloud, 255, 255, 255);
	viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud_trans, 230, 20, 20); // Red
	viewer.addPointCloud (cloud_trans, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
	}
}

void simple_icp(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, 
	Eigen::Matrix4f * trans_ptr)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt);

	PointCloud Final;
	icp.setMaximumIterations(5000);
	icp.align(Final);

	*trans_ptr = icp.getFinalTransformation();
}

void simple_sac(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, 
	pcl::Correspondences * all_corres, pcl::Correspondences * inliers_corres,  
	Eigen::Matrix4f * trans_ptr, int interation_num=10000)
{
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;

	sac.setInputSource(cloud_src);
	sac.setInputTarget(cloud_tgt);

	sac.setMaximumIterations(interation_num);
	sac.getRemainingCorrespondences(*all_corres, *inliers_corres);
	
	*trans_ptr = sac.getBestTransformation();	
	// std::cout << "Transformation from Ransac: " << std::endl;
	// std::cout << sac.getBestTransformation() << std::endl << std::endl;

}

void simple_CorrEst(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,
	pcl::Correspondences * all_corres, pcl::Correspondences * inliers_corres)
{
	pcl::registration::CorrespondenceEstimation<PointT, PointT> est;
	/*  
	Correspondence:
		http://docs.pointclouds.org/trunk/correspondence_8h_source.html

	Correspondences:
		typedef std::vector< pcl::Correspondence, 
		Eigen::aligned_allocator<pcl::Correspondence> > Correspondences;
	*/

	est.setInputSource (cloud_src);
	est.setInputTarget (cloud_tgt);

	est.determineCorrespondences (*all_corres);

	// Display: correspondences
	// for (pcl::Correspondences::iterator it = (*all_corres).begin(); it != (*all_corres).end(); ++it)
	// {
	// 	std::cout << ' ' << (*it).index_query << ", " << (*it).index_match << ", " << (*it).distance;
	// 	std::cout << std::endl;
	// }
}

void simple_svd(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, 
	const pcl::Correspondences & all_corres, Eigen::Matrix4f * trans_ptr)
{
	pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;

	svd.estimateRigidTransformation(*cloud_src, *cloud_tgt, 
		all_corres, *trans_ptr);

	// std::cout << "Transformation from SVD: " << std::endl;
	// std::cout << *trans_ptr << std::endl << std::endl;
}

int main(int argc, char const *argv[])
{
	PointCloudPtr cloud_in (new PointCloud);
	PointCloudPtr cloud_out (new PointCloud);

	pcl::Correspondences all_correspondences;
	pcl::Correspondences inliers;

	// toy data
	generate_cloud(cloud_in, 100, 1, true, false);
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;

	/* Regid Transformation */ 
	transform_cloud(cloud_out);

	/* Visualization */
	// vis_clouds2(cloud_in, cloud_out);

	/* Keypoint Detection */
	// to-do

	/* Feature Descriptors */
	// to-do

	/* Correspondence Estimation */
	// to-do: k-d tree method
	simple_CorrEst(cloud_in, cloud_out, &all_correspondences, &inliers);

	/* Transformation Estimation SVD */
	Eigen::Matrix4f *transformation_svd (new Eigen::Matrix4f);
	const pcl::Correspondences & all_corr = all_correspondences;

	simple_svd(cloud_in, cloud_out, all_corr, transformation_svd);
	
	std::cout << "Transformation from SVD: " << std::endl;
	std::cout << *transformation_svd << std::endl << std::endl;

	/* SAC section */
	Eigen::Matrix4f *transformation_sac_ptr (new Eigen::Matrix4f); 
	simple_sac(cloud_in, cloud_out, &all_correspondences, &inliers, transformation_sac_ptr);
	std::cout << "Transformation from Ransac: " << std::endl;
	std::cout << *transformation_sac_ptr << std::endl << std::endl;

	/* ICP section */
	Eigen::Matrix4f *transformation_icp_ptr (new Eigen::Matrix4f); 
	simple_icp(cloud_in, cloud_out, transformation_icp_ptr);
	std::cout << "Transformation from ICP: " << std::endl;
	std::cout << *transformation_icp_ptr << std::endl;

	// to-do: Method2: NDT

	return 0;
}

/*
	Helpful links:
	https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
	https://stackoverflow.com/questions/1143262/what-is-the-difference-between-const-int-const-int-const-and-int-const
*/