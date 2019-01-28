#include "typedefs.h"
#include "utils.h"

void filter(const PointCloud::Ptr cloud_src, \
  const PointCloud::Ptr filtered, float &nn_time, float &bd_time)
{
    int num = cloud_src->size();
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_src);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*filtered);

    // sor.getNnTime(nn_time);
    // sor.getBuildTime(bd_time);
    // std::cout << "Outlier Filtering:" << std::endl;
    // std::cout << (num - filtered->size()) << " Points Are Filtered: " \
    // << num << "->" << filtered->size() << std::endl << std::endl;
}

void downSample(const PointCloud::Ptr cloud_src, \
  const PointCloud::Ptr filtered, float gridsize)
{
    pcl::VoxelGrid<PointT> grid; //VoxelGrid
    grid.setLeafSize(gridsize, gridsize, gridsize);
    std::cout << "Before Downsampling:" << cloud_src->size() << std::endl;
    grid.setInputCloud(cloud_src);
    grid.filter(*filtered); 
    std::cout << "After Downsampling: " << filtered->size() << std::endl;
}

void detectKeyPoints(std::string kernel_keyPoints, const PointCloud::Ptr cloud_ptr, const PointCloud::Ptr keyPoints_cal_ptr)
{
	if (kernel_keyPoints == "NARF")
	{
        std::cout << "NARF Keypoints Are Being Detected!" << std::endl;
		keyPointsNARF(cloud_ptr, keyPoints_cal_ptr);
	}
	if (kernel_keyPoints == "SIFT")
	{
        std::cout << "SIFT Keypoints Are Being Detected!" << std::endl;
		keyPointsSIFT(cloud_ptr, keyPoints_cal_ptr);
	}
	if (kernel_keyPoints == "HARRIS")
	{
        std::cout << "HARRIS Keypoints Are Being Detected!" << std::endl;
		keyPointsHARRIS(cloud_ptr, keyPoints_cal_ptr);
	}
/*	if (kernel_keyPoints == "LOAM")
	{
        std::cout << "LOAM Keypoints Are Being Detected!" << std::endl;
		keyPointsLOAM(cloud_ptr, keyPoints_cal_ptr);
	}*/
}

void describeFeatures(std::string kernel_descriptors, FeatureCloud &cloud, float R, \
    float &nn_time, float &bd_time)
{
	if (kernel_descriptors == "FPFH")
	{
        std::cout << "FPFH Descriptors Are Being Detected!" << std::endl;
        computeFeatures_FPFH (cloud, R, nn_time, bd_time);
	}
	if (kernel_descriptors == "VFH")
	{
        std::cout << "VFH Descriptors Are Being Detected!" << std::endl;
		computeFeatures_VFH (cloud, R);
	}
	if (kernel_descriptors == "SpinImage")
	{
        std::cout << "SpinImage Descriptors Are Being Detected!" << std::endl;
		computeFeatures_SpinImage (cloud, R);
	}
	if (kernel_descriptors == "3DSC")
	{
        std::cout << "3DSC Descriptors Are Being Detected!" << std::endl;
		computeFeatures_3DSC (cloud, R, nn_time, bd_time);
	}
	// if (kernel_descriptors == "USC")
	// {
 //        std::cout << "USC Descriptors Are Being Detected!" << std::endl;
	// 	computeFeatures_USC (cloud, R);
	// }
	if (kernel_descriptors == "SHOT")
	{
        std::cout << "SHOT Descriptors Are Being Detected!" << std::endl;
		computeFeatures_SHOT (cloud, R, nn_time, bd_time);
	}
	// if (kernel_descriptors == "RSD")
	// {
 //        std::cout << "RSD Descriptors Are Being Detected!" << std::endl;
	// 	computeFeatures_RSD (cloud, R);
	// }
}

void pointsNumberCheck(FeatureCloud &cloud)
{
	PointCloud::Ptr nanremoved_(new PointCloud);
	PointCloud::Ptr cloud_ = cloud.getPointCloud();
	SurfaceNormals::Ptr normals_ = cloud.getSurfaceNormals();
	std::vector<int> index;

    int num_point = cloud_->size();
    int num_normal = normals_->size();

	pcl::removeNaNNormalsFromPointCloud(*normals_, *normals_, index);
	pcl::copyPointCloud(*cloud_, index, *nanremoved_);
    cloud.setInputCloud(nanremoved_);

    // std::cout << "Remove NAN From Normals:" << std::endl;
    // std::cout << (num_normal - normals_->size()) << " Normals Are Removed: " \
    // << num_normal << "->" << normals_->size() << std::endl;
    // std::cout << (num_point - nanremoved_->size()) << " Points Are Removed: " \
    // << num_point << "->" << nanremoved_->size() << std::endl << std::endl;

}

void removeNANFromFPFH(FPFH_Features::Ptr feature_descriptor, FPFH_Features::Ptr nanremoved, FeatureCloud &cloud)
{
    PointCloud::Ptr keyPoints_nanremoved_ptr(new PointCloud);
    pcl::PointIndices::Ptr indices_nanremoved_ptr (new pcl::PointIndices);

    for (int i=0; i<feature_descriptor->points.size();i++)
    {
        float p = feature_descriptor->points[i].histogram[0];
        if (p != p) 
        {
            continue;
        }
        else
        {
            nanremoved->push_back(feature_descriptor->points[i]);
            keyPoints_nanremoved_ptr->push_back(cloud.getKeyPoints()->points[i]);
            indices_nanremoved_ptr->indices.push_back((cloud.getKeyPoint_indices())->indices[i]);
        }
    }
    if(feature_descriptor->points.size() != nanremoved->points.size())
    {
   		cloud.setKeyPoints(keyPoints_nanremoved_ptr);
    	cloud.setKeyPoint_indices(indices_nanremoved_ptr);
    }

    // std::cout << "Remove NAN From Feature Descriptors:" << std::endl;
    // std::cout << (feature_descriptor->points.size() - nanremoved->points.size()) << " Feature Descriptors Are Removed: " \
    // << feature_descriptor->points.size() << "->" << nanremoved->points.size() << std::endl << std::endl;
}

// Narf Key Point Detection
void keyPointsNARF(const PointCloud::Ptr cloud_src, \
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

    PointCloud& keypoints = *keyPoints_NARF;
    keypoints.points.resize (keypoint_indices.points.size ());
    for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    {
        keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
        // std::cout << keypoint_indices.points[i] << ", "<< keypoints.points[i].index << std::endl;
    }
    std::cout << keypoint_indices.points.size () << " NARF Key Points Are Detected." << std::endl << std::endl;
}
 
void keyPointsSIFT(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_SIFT)
{
    const float min_scale = 0.03f; //the standard deviation of the smallest scale in the scale space   //0.005
    const int n_octaves = 8; //the number of octaves (i.e. doublings of scale) to compute         //6
    const int n_scales_per_octave = 12;//the number of scales to compute within each octave          //4
    const float min_contrast = 0.08f;//the minimum contrast required for detection             //0.005

    // Estimate the sift interest points using z values from xyz as the Intensity variants
    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);

    sift.setInputCloud(cloud_src);
    sift.compute(result);
    copyPointCloud(result, *keyPoints_SIFT);
    
    // std::cout << result.points.size() << " SIFT Key Points Are Detected." << std::endl << std::endl;
}

void keyPointsHARRIS(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_Harris)
{
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
    cloud_out->resize(cloud_out->height*cloud_src->width);
    cloud_out->clear();
    harris.compute(*cloud_out);
    int size = cloud_out->size();

    keyPoints_Harris->height = 1;
    keyPoints_Harris->width = 100;
    keyPoints_Harris->resize(cloud_out->height*cloud_src->width);
    keyPoints_Harris->clear();

    pcl::PointXYZ point;
    for (int i = 0; i<size; i++)
    {
        point.x = cloud_out->at(i).x;
        point.y = cloud_out->at(i).y;
        point.z = cloud_out->at(i).z;
        keyPoints_Harris->push_back(point);
    }
    // std::cout << keyPoints_Harris->size() << " HARRIS Key Points Are Detected." << std::endl << std::endl;
}

// only used for parameters finetune
void keyPointsSIFTtest(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_SIFT, float min_scale, int n_octaves, int n_scales_per_octave, float min_contrast)
{
/*    const float min_scale = 0.1f; //the standard deviation of the smallest scale in the scale space   //0.005
    const int n_octaves = 6; //the number of octaves (i.e. doublings of scale) to compute         //6
    const int n_scales_per_octave = 4;//the number of scales to compute within each octave          //4
    const float min_contrast = 0.1f;//the minimum contrast required for detection             //0.005*/

    std::cout << "min_scale: " << min_scale << std::endl;
    std::cout << "n_octaves: " << n_octaves << std::endl;
    std::cout << "n_scales_per_octave: " << n_scales_per_octave << std::endl;
    std::cout << "min_contrast: " << min_contrast << std::endl;


    // Estimate the sift interest points using z values from xyz as the Intensity variants
    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);

    sift.setInputCloud(cloud_src);
    sift.compute(result);
    copyPointCloud(result, *keyPoints_SIFT);

    std::cout << result.points.size() << " SIFT Key Points Are Detected." << std::endl << std::endl;
}

// only used for parameters finetune
void keyPointsHARRIStest(const PointCloud::Ptr cloud_src, const PointCloud::Ptr keyPoints_Harris, float radius, float threshold)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;

    harris.setInputCloud(cloud_src);

    // Filter only
    harris.setRadius(radius);//0.5
    harris.setThreshold(threshold);//0.02
    harris.setNonMaxSupression(true);
    harris.setRefine(true);

    cloud_out->height = 1;
    cloud_out->width = 100;
    cloud_out->resize(cloud_out->height*cloud_src->width);
    cloud_out->clear();
    harris.compute(*cloud_out);
    int size = cloud_out->size();

    keyPoints_Harris->height = 1;
    keyPoints_Harris->width = 100;
    keyPoints_Harris->resize(cloud_out->height*cloud_src->width);
    keyPoints_Harris->clear();

    pcl::PointXYZ point;
    for (int i = 0; i<size; i++)
    {
        point.x = cloud_out->at(i).x;
        point.y = cloud_out->at(i).y;
        point.z = cloud_out->at(i).z;
        keyPoints_Harris->push_back(point);
    }
    std::cout << keyPoints_Harris->size() << " HARRIS Key Points Are Detected." << std::endl << std::endl;
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
		std::cout << path_dir << std::endl;
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
	std::cout << "total number of frames: " << file_num << std::endl << std::endl;

	// sort
	std::sort (files.begin(), files.end(), string_sort);
}

void load_bin(std::string infile, FeatureCloud &cloud)
{
    std::cout << std::endl;
	std::cout << "Loading " << infile << std::endl;

	std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);

	if(!input.good()){
		std::cerr << "Could not read file: " << infile << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);

	/* convertion */
	PointCloud::Ptr points (new PointCloud);

	for (int j=0; input.good() && !input.eof(); j++) {
		//PointXYZI point;
		PointT point, point_gt;

		input.read((char *) &point.x, 3*sizeof(float));
		input.seekg(sizeof(float), std::ios::cur);    

		/* coordinate system trans*/
		point_gt.x = -point.y;
		point_gt.y = -point.z;
		point_gt.z = point.x;

		points->push_back(point_gt);
	}

	cloud.setInputCloud(points);
	input.close();

    std::cout << "Number Of Points Loaded: " << points->points.size() << std::endl << std::endl;
}

// Compute the surface normals
void computeSurfaceNormals (FeatureCloud &cloud, float R, \
    float &nn_time, float &bd_time)
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
    // norm_est.getNnTime(nn_time);
    // norm_est.getBuildTime(bd_time);

    cloud.setSurfaceNormals(normals_);
    // std::cout << "Normals:" << *normals_ << std::endl;
}

void computeFeatures_FPFH (FeatureCloud &cloud, float R, float &nn_time, float &bd_time)
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

    // fpfh_est.getNnTime(nn_time);
    // fpfh_est.getBuildTime(bd_time);

    removeNANFromFPFH(fpfh_features_, nanremoved_, cloud);

    // for future optimization
    //removeNANFromDescriptor<FPFH_Features::Ptr>(fpfh_features_, nanremoved_, cloud);

    cloud.setFeatures_FPFH(nanremoved_);
}

void computeFeatures_VFH (FeatureCloud &cloud, float R)
{
	VFH_Features::Ptr vfh_features_(new VFH_Features);
	pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh_est;
	vfh_est.setSearchSurface(cloud.getPointCloud());
	vfh_est.setInputNormals(cloud.getSurfaceNormals());
	vfh_est.setInputCloud(cloud.getKeyPoints());

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	vfh_est.setSearchMethod(tree);
	vfh_est.setRadiusSearch(R);

	vfh_est.compute(*vfh_features_);
	cloud.setFeatures_VFH(vfh_features_);

	//std::cout << "output points.size (): " << vfh_features_->points.size() << std::endl;
}

void computeFeatures_SpinImage (FeatureCloud &cloud, float R)
{
	SpinImage_Features::Ptr spinImage_features_(new SpinImage_Features);
	pcl::SpinImageEstimation<PointT, pcl::Normal, pcl::Histogram<153>> spinimage_est;
	spinimage_est.setSearchSurface(cloud.getPointCloud());
	spinimage_est.setInputNormals(cloud.getSurfaceNormals());
	spinimage_est.setInputCloud(cloud.getKeyPoints());


	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	spinimage_est.setSearchMethod(tree);
	spinimage_est.setRadiusSearch(R);

	spinimage_est.compute(*spinImage_features_);
	cloud.setFeatures_SpinImage(spinImage_features_);

	// std::cout << "output points.size (): " << spinImage_features_->points.size() << std::endl;
}

void computeFeatures_3DSC (FeatureCloud &cloud, float R, float &nn_time, float &bd_time)
{
	threeDSC_Features::Ptr threedsc_features_(new threeDSC_Features);
    threeDSC_Features::Ptr nanremoved_(new threeDSC_Features);
	pcl::ShapeContext3DEstimation<PointT, pcl::Normal, pcl::ShapeContext1980> threedsc_est;
	threedsc_est.setSearchSurface(cloud.getPointCloud());
	threedsc_est.setInputNormals(cloud.getSurfaceNormals());
	threedsc_est.setInputCloud(cloud.getKeyPoints());

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	threedsc_est.setSearchMethod(tree);
	threedsc_est.setRadiusSearch(R);

	// The minimal radius value for the search sphere, to avoid being too sensitive
	// in bins close to the center of the sphere.
	threedsc_est.setMinimalRadius(0.4);
	// Radius used to compute the local point density for the neighbors
	// (the density is the number of points within that radius).
	threedsc_est.setPointDensityRadius(0.45);


	threedsc_est.compute(*threedsc_features_);

    	// threedsc_est.getNnTime(nn_time);
    	// threedsc_est.getBuildTime(bd_time);

    removeNANFromDescriptor<threeDSC_Features::Ptr>(threedsc_features_, nanremoved_, cloud);

	cloud.setFeatures_3DSC(nanremoved_);
}

// void computeFeatures_USC (FeatureCloud &cloud, float R)
// {
// 	USC_Features::Ptr usc_features_(new USC_Features);
//     USC_Features::Ptr nanremoved_(new USC_Features);
// 	pcl::UniqueShapeContext<PointT, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc_est;
// 	usc_est.setSearchSurface(cloud.getPointCloud());
// 	//usc_est.setInputNormals(cloud.getSurfaceNormals());
// 	usc_est.setInputCloud(cloud.getKeyPoints());

// 	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

// 	usc_est.setSearchMethod(tree);
// 	usc_est.setRadiusSearch(R);

// 	// The minimal radius value for the search sphere, to avoid being too sensitive
// 	// in bins close to the center of the sphere.
//     usc_est.setMinimalRadius(0.4);
// 	// Radius used to compute the local point density for the neighbors
// 	// (the density is the number of points within that radius).
//     usc_est.setPointDensityRadius(0.45);
// 	// Set the radius to compute the Local Reference Frame.
//     usc_est.setLocalRadius(0.4);

// 	usc_est.compute(*usc_features_);

//     removeNANFromDescriptor<USC_Features::Ptr>(usc_features_, nanremoved_, cloud);

// 	cloud.setFeatures_USC(nanremoved_);
// }


void computeFeatures_SHOT (FeatureCloud &cloud, float R, float &nn_time, float &bd_time)
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

    	// shot_est.getNnTime(nn_time);
    	// shot_est.getBuildTime(bd_time);

	//removeNANFromSHOT(shot_features_, nanremoved_, cloud);

    //for future optimization
    removeNANFromDescriptor<SHOT_Features::Ptr>(shot_features_, nanremoved_, cloud);

	cloud.setFeatures_SHOT(nanremoved_);
}

// strange output, only 2 dimensions.
// void computeFeatures_RSD (FeatureCloud &cloud, float R)
// {
// 	RSD_Features::Ptr rsd_features_(new RSD_Features);
// 	pcl::RSDEstimation<PointT, pcl::Normal, RSD_FeatureT> rsd_est;
// 	rsd_est.setSearchSurface(cloud.getPointCloud());
// 	rsd_est.setInputNormals(cloud.getSurfaceNormals());
// 	rsd_est.setInputCloud(cloud.getKeyPoints());

// 	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

// 	rsd_est.setSearchMethod(tree);
// 	rsd_est.setRadiusSearch(R);

// 	rsd_est.compute(*rsd_features_);
// 	rsd_est.setSaveHistograms(true);
// 	// Plane radius. Any radius larger than this is considered infinite (a plane).
// 	rsd_est.setPlaneRadius(1.0);

// 	cloud.setFeatures_RSD(rsd_features_);

// 	std::cout << "output points.size (): " << rsd_features_->points.size() << std::endl;
	
// 	ofstream RSDSignature;
// 	RSDSignature.open("RSDSignature.txt");
// 	for (int i = 0; i < rsd_features_->points.size(); i++)
// 	{
// 	 	RSD_FeatureT descriptor = rsd_features_->points[i];
// 	 	//std::cout << descriptor << std::endl;
// 	 	RSDSignature << descriptor << endl;
// 		//std::cout << rsd_est.getHistograms() << std::endl;
// 	}
// 	RSDSignature.close();
// }


// Initialize the PointCloudNormal type
void construct_PointNormal(FeatureCloud &source_cloud, FeatureCloud &target_cloud, \
  int use_keypoints)
{
	// src
	PointCloudNormal::Ptr pointNormal_src (new PointCloudNormal);
	PointCloudNormal::Ptr pointNormal_tgt (new PointCloudNormal);

	if (use_keypoints == 1){
	// to-do
	}
  	else{
    	// src
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

void correspondence_estimation(std::string kernel_descriptors, std::string flag_reciprocal, \
    FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
    pcl::Correspondences &all_corres, float &nn_time, float &bd_time)
{ 
	if (kernel_descriptors == "FPFH")
	{
		pcl::registration::CorrespondenceEstimation<FPFH_FeatureT, FPFH_FeatureT> est;
        est.setInputSource (source_cloud.getFeatures_FPFH());
        est.setInputTarget (target_cloud.getFeatures_FPFH());
		if(flag_reciprocal == "True")
			est.determineReciprocalCorrespondences (all_corres);
        	// est.getBuildTime(bd_time);
        	// est.getNnTime(nn_time);
	}
	if (kernel_descriptors == "VFH")
	{
		pcl::registration::CorrespondenceEstimation<VFH_FeatureT, VFH_FeatureT> est;
		est.setInputSource (source_cloud.getFeatures_VFH());
		est.setInputTarget (target_cloud.getFeatures_VFH());
		if(flag_reciprocal == "True")
			est.determineReciprocalCorrespondences (all_corres);
       		// est.getBuildTime(bd_time);
        	// est.getNnTime(nn_time);
	}
/*	if (kernel_descriptors == "SpinImage")
	{
        pcl::registration::CorrespondenceEstimation<SpinImage_FeatureT, SpinImage_FeatureT> est;
        est.setInputSource (source_cloud.getFeatures_SpinImage());
        est.setInputTarget (target_cloud.getFeatures_SpinImage());
        if(flag_reciprocal == "True")
            est.determineReciprocalCorrespondences (all_corres);
	}*/
	if (kernel_descriptors == "3DSC")
	{
		pcl::registration::CorrespondenceEstimation<threeDSC_FeatureT, threeDSC_FeatureT> est;
		est.setInputSource (source_cloud.getFeatures_3DSC());
		est.setInputTarget (target_cloud.getFeatures_3DSC());
		if(flag_reciprocal == "True")
			est.determineReciprocalCorrespondences (all_corres);
        	// est.getBuildTime(bd_time);
        	// est.getNnTime(nn_time);
	}
	// if (kernel_descriptors == "USC")
	// {
	// 	pcl::registration::CorrespondenceEstimation<USC_FeatureT, USC_FeatureT> est;
	// 	est.setInputSource (source_cloud.getFeatures_USC());
	// 	est.setInputTarget (target_cloud.getFeatures_USC());
	// 	if(flag_reciprocal == "True")
	// 		est.determineReciprocalCorrespondences (all_corres);
	// }
	if (kernel_descriptors == "SHOT")
	{
		pcl::registration::CorrespondenceEstimation<SHOT_FeatureT, SHOT_FeatureT> est;
		est.setInputSource (source_cloud.getFeatures_SHOT());
        est.setInputTarget (target_cloud.getFeatures_SHOT());

		if(flag_reciprocal == "True")
			est.determineReciprocalCorrespondences (all_corres);
	
        	// est.getBuildTime(bd_time);
        	// est.getNnTime(nn_time);
    }
	// if (kernel_descriptors == "RSD")
	// {
	// 	pcl::registration::CorrespondenceEstimation<RSD_FeatureT, RSD_FeatureT> est;
	// 	est.setInputSource (source_cloud.getFeatures_RSD());
	// 	est.setInputTarget (target_cloud.getFeatures_RSD());
	// 	if(flag_reciprocal == "True")
	// 		est.determineReciprocalCorrespondences (all_corres);
	// }
}


// previous XYZ based rejection
void correspondences_rejection(FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  pcl::Correspondences &correspondences, pcl::Correspondences &inliers, \
  Result *result, int N, float threshold, int use_keypoints)
{

	pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
	sac.setInputSource(source_cloud.getKeyPoints ());
	sac.setInputTarget(target_cloud.getKeyPoints ());

	// sac.setInputSource(source_cloud.getPointCloud ());
	// sac.setInputTarget(target_cloud.getPointCloud ());

	// Set the threshold for rejection iteration
	sac.setInlierThreshold(threshold);
	sac.setMaximumIterations(N);
	sac.getRemainingCorrespondences(correspondences, inliers);

	Eigen::Matrix4f transformation = sac.getBestTransformation();

	result->final_transformation = transformation;

	// std::cout << "RANSAC:" << std::endl;
	// for(int i = 0; i < 3; i ++)
	// {
	// 	for(int j = 0; j < 4; j ++)
	// 	{
	// 		std::cout << ("%lf", transformation(i,j));
	// 		if (j != 4)
	// 			std::cout << " ";
	// 	}
	// 	std::cout << std::endl;
	// }

	PointCloud::Ptr transformed_cloud (new PointCloud);

	if (use_keypoints == 1)
	{
		pcl::transformPointCloud (*(source_cloud.getKeyPoints ()), *transformed_cloud, transformation);
	}
	else
	{
		pcl::transformPointCloud (*(source_cloud.getPointCloud ()), *transformed_cloud, transformation);
	}
	source_cloud.setTransformedCloud(transformed_cloud);

    // std::cout << "Correspondences Rejection:" << std::endl;
    // std::cout << (correspondences.size() - inliers.size()) << " Correspondences Are Rejected: " \
    // << correspondences.size() << "->" << inliers.size() << std::endl << std::endl;
}

void iterative_closest_points(std::string solver, std::string flag_reciprocal, \
    std::string flag_ransac, FeatureCloud &source_cloud, FeatureCloud &target_cloud,\
  Result *result, float transEps, float corresDist, float EuclFitEps, float outlThresh, \
  int use_keypoint, pcl::Correspondences &inliers, float &nn_time, float &bd_time)
{
    PointCloudNormal Final;

    PointCloudNormal::Ptr pointNormal_src = source_cloud.getPointCloudNormal();
    PointCloudNormal::Ptr pointNormal_tgt = target_cloud.getPointCloudNormal();

    pcl::registration::CorrespondenceRejector::Ptr ransac_rej \
    (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointNormal> ()); 

    if(solver == "SVD")
    {
        std::cout << "SVD Solver for ICP Is Running!" << std::endl;
    	pcl::IterativeClosestPoint<PointNormal, PointNormal> icp;

        if (flag_reciprocal == "True")
            icp.setUseReciprocalCorrespondences(true);
        if (flag_ransac == "True")
        {
            icp.setRANSACOutlierRejectionThreshold (outlThresh);
            // add ransac rejector
            icp.addCorrespondenceRejector(ransac_rej);
        }

    	icp.setInputSource(pointNormal_src);  
    	icp.setInputTarget(pointNormal_tgt);

    	icp.setMaximumIterations (15);
    	icp.setTransformationEpsilon (transEps);
    	icp.setMaxCorrespondenceDistance (corresDist);
    	icp.setEuclideanFitnessEpsilon (EuclFitEps);

    	icp.align(Final);

    	result->final_transformation = icp.getFinalTransformation();
    	result->fitness_score = icp.getFitnessScore();

        // icp.getNnTime_icp(nn_time);
        // icp.getBuildTime_icp(bd_time);

    }
    if (solver == "LM")
    {
        std::cout << "LM Solver for ICP Is Running!" << std::endl;
        pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> icp_lm;

        if (flag_reciprocal == "True")
            icp_lm.setUseReciprocalCorrespondences(true);
        if (flag_ransac == "True")
        {
            icp_lm.setRANSACOutlierRejectionThreshold (outlThresh);
            // add ransac rejector
            icp_lm.addCorrespondenceRejector(ransac_rej);
        }

        icp_lm.setInputSource(pointNormal_src);
        icp_lm.setInputTarget(pointNormal_tgt);

        icp_lm.setMaximumIterations(15);
        icp_lm.setTransformationEpsilon(transEps);
        icp_lm.setMaxCorrespondenceDistance(corresDist);
        icp_lm.setEuclideanFitnessEpsilon (EuclFitEps);

        icp_lm.align(Final);

        result->final_transformation = icp_lm.getFinalTransformation();
        result->fitness_score = icp_lm.getFitnessScore();

        // icp_lm.getNnTime_icp(nn_time);
        // icp_lm.getBuildTime_icp(bd_time);

    }
}

void keyPoint_dist(FeatureCloud &source_cloud, FeatureCloud &target_cloud)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target_cloud.getKeyPoints());

    PointCloud::Ptr keypoints_cal = source_cloud.getKeyPoints();

    std::vector<float>pointNKNSquareDistance;
    std::vector<int> pointIdxNKNSearch;

    int indice = 0;

    for (size_t i =0; i < keypoints_cal->size();i++)
    {
        kdtree.nearestKSearch(keypoints_cal->points[i],1,pointIdxNKNSearch,pointNKNSquareDistance);

        std::cout << "Distances:"<<pointNKNSquareDistance[0] << "," << \
        pointNKNSquareDistance[1]<< "," << \ 
        pointNKNSquareDistance[2]<< "," << std::endl;
        // std::cout<<"the indices are:"<<pointIdxNKNSearch[0]<<endl;
        indice = pointIdxNKNSearch[0];
        // indices->indices.push_back(indice);
        // keypoints_real->points.push_back(cloudin->points[indice]);
    }
}
