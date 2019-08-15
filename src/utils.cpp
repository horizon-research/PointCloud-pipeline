#include "typedefs.h"
#include "utils.h"
#include "modules.h"
#include "read_config.h"

/** global variable definition**/
std::string g_DATASET_DIR;	// Dataset directory
std::string g_RESULT_DIR;	// Result directory

// Normal Estimation Parameters
float g_NORMAL_SEARCH_RADIUS;
bool g_NORMAL_USE_CUSTOMIZED_KDTREE;
int g_NORMAL_MAX_LEAF_SIZE;

// Key Point Detection Parameters
std::string g_KEYPOINTS_MODULE;

// Feature Description Parameters
float g_FEATURE_SEARCH_RADIUS;
std::string g_FEATURE_MODULE;

// Correspondence Estimation Parameters
bool g_CORR_EST_USE_RECIPROCAL_SEARCH;

// Correspondence Rejection Parameters
float g_RANSAC_THRESHOLD;
int g_RANSAC_MAX_ITERATION;

// ICP Parameters
std::string g_ICP_SOLVER;
int g_ICP_MAX_ITERATION;
bool g_ICP_USE_RANSAC;
bool g_ICP_USE_RECIPROCAL_SEARCH;

float g_ICP_TRANSFORMATION_EPSILON;
float g_ICP_MAX_CORRESPONDENCE_DISTANCE;
float g_ICP_EUCLIDEAN_FITNESS_EPSILON;
float g_ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD;

bool g_ICP_USE_CUSTOMIZED_KDTREE;
int g_ICP_MAX_LEAF_SIZE;

float g_APPROX_RADIUS_SEARCH_PARA;
float g_APPROX_NEAREST_SEARCH_PARA;

bool g_SAVE_APPROX_DATA;

void readConfigFile(std::string file_name)
{
	Config config_file(file_name);

	// File Directory Parameters
	g_DATASET_DIR = config_file.Read("Dataset_Directory", g_DATASET_DIR);
	g_RESULT_DIR = config_file.Read("Result_Directory", g_RESULT_DIR);

	// Normal Estimation Parameters
	g_NORMAL_SEARCH_RADIUS = config_file.Read("Normal_Search_Radius", g_NORMAL_SEARCH_RADIUS);
	g_NORMAL_USE_CUSTOMIZED_KDTREE = config_file.Read("Normal_Use_Customized_KDTree", g_NORMAL_USE_CUSTOMIZED_KDTREE);
	g_NORMAL_MAX_LEAF_SIZE = config_file.Read("Normal_Max_Leaf_Size", g_NORMAL_MAX_LEAF_SIZE);

	// Key Point Detection Parameters
	g_KEYPOINTS_MODULE = config_file.Read("Key_Point_Detection_Module", g_KEYPOINTS_MODULE);

	// Feature Description Parameters
	g_FEATURE_SEARCH_RADIUS = config_file.Read("Feature_Search_Radius", g_FEATURE_SEARCH_RADIUS);
	g_FEATURE_MODULE = config_file.Read("Feature_Module", g_FEATURE_MODULE);

	// Correspondence Estimation Parameters
	g_CORR_EST_USE_RECIPROCAL_SEARCH = config_file.Read("Corr_Est_Use_Reciprocal_Search", g_CORR_EST_USE_RECIPROCAL_SEARCH);

	// Correspondence Rejection Parameters
	g_RANSAC_THRESHOLD = config_file.Read("Ransac_Threshold", g_RANSAC_THRESHOLD);
	g_RANSAC_MAX_ITERATION = config_file.Read("Ransac_Max_Iteration", g_RANSAC_MAX_ITERATION);

	// ICP Parameters
	g_ICP_SOLVER = config_file.Read("ICP_Solver", g_ICP_SOLVER);
	g_ICP_MAX_ITERATION = config_file.Read("ICP_Max_Iteration", g_ICP_MAX_ITERATION);
	g_ICP_USE_RANSAC = config_file.Read("ICP_Use_Ransac", g_ICP_USE_RANSAC);
	g_ICP_USE_RECIPROCAL_SEARCH = config_file.Read("ICP_Use_Reciprocal_Search", g_ICP_USE_RECIPROCAL_SEARCH);
	
	g_ICP_TRANSFORMATION_EPSILON = config_file.Read("ICP_Transformation_Epsilon", g_ICP_TRANSFORMATION_EPSILON);
	g_ICP_MAX_CORRESPONDENCE_DISTANCE = config_file.Read("ICP_Max_Correspondence_Distance", g_ICP_MAX_CORRESPONDENCE_DISTANCE);
	g_ICP_EUCLIDEAN_FITNESS_EPSILON = config_file.Read("ICP_Euclidean_Fitness_Epsilon", g_ICP_EUCLIDEAN_FITNESS_EPSILON);
	g_ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD = config_file.Read("ICP_Ransac_Outlier_Rejection_Threshold", g_ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD);

	g_ICP_USE_CUSTOMIZED_KDTREE = config_file.Read("ICP_Use_Customized_KDTree", g_ICP_USE_CUSTOMIZED_KDTREE);
	g_ICP_MAX_LEAF_SIZE = config_file.Read("ICP_Max_Leaf_Size", g_ICP_MAX_LEAF_SIZE);

	// Approximation Search Parameters
	g_APPROX_RADIUS_SEARCH_PARA = config_file.Read("Approx_Radius_Search_Para", g_APPROX_RADIUS_SEARCH_PARA);
	g_APPROX_NEAREST_SEARCH_PARA = config_file.Read("Approx_Nearest_Search_Para", g_APPROX_NEAREST_SEARCH_PARA);

	g_SAVE_APPROX_DATA = config_file.Read("Save_Approx_Data", g_SAVE_APPROX_DATA);

	std::cout << std::endl << "Reading Config Successfully!" << std::endl;
}

void makeResultFolder(std::string &g_result_dir)
{
	DIR *dirp;
	struct dirent *directory;

	int last_file_id = 0;
	std::string cmd;
	std::vector<std::string> file_name;

	cmd = "mkdir -p " + g_result_dir;
        system(cmd.c_str());

	dirp = opendir(g_result_dir.c_str());
	if (dirp)
	{
		while ((directory = readdir(dirp)) != NULL)
		{
			file_name.push_back(directory->d_name);
		}

		closedir(dirp);

		if (file_name.size() == 2)
		{
			cmd = "mkdir " + g_result_dir+ "result_1";
		}
		else
		{
			for (int i = 0; i < file_name.size(); i++)
			{
				if (file_name[i].find("result") != std::string::npos)
				{
					int split_index = file_name[i].find("_");
					int cur_file_id = std::stoi(file_name[i].substr(split_index + 1));
					if (last_file_id < cur_file_id)
						last_file_id = cur_file_id;
				}

			}
			cmd = "mkdir " + g_result_dir + "result_" + std::to_string(last_file_id + 1);
		}

		g_result_dir = g_result_dir + "result_" + std::to_string(last_file_id + 1) + "/";

		if (!system(cmd.c_str()))
			std::cout << "Making New Directory Successfully!" << std::endl;
		else
		{
			std::cout << "Making New Result Directory Failed!" << std::endl;
			exit(-1);
		}
	}
	else {
		std::cout << errno << std::endl;
	}
}

bool sortString (std::string i,std::string j) 
{
	// the part before '.bin' 
	std::string foo_i = i.substr(0,i.length()-4);
	std::string foo_j = j.substr(0,i.length()-4);
	return (foo_i<foo_j); 
}

void listDir(std::string path_dir, std::vector<std::string> &files)
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
	std::sort (files.begin(), files.end(), sortString);
}

void loadBINFile(std::string infile, FeatureCloud &cloud)
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

void loadTXTFile(std::string infile, FeatureCloud &cloud)
{
    std::cout << "Loading " << infile << std::endl;

    std::ifstream file(infile.c_str());
    std::string str;

    PointCloud::Ptr points (new PointCloud);
    std::vector<std::string> raw_data;

    while (std::getline(file, str))
        raw_data.push_back(str);
    std:: cout << "Size: " << raw_data.size() << std::endl;

    for (int i = 0; i < raw_data.size(); i++)
    {
        PointT point;

        std::stringstream raw_ss(raw_data[i]);

        int j = 0;
        while (raw_ss.good())
        {
           std::string substr;
           std::getline(raw_ss, substr, ',');
           float value_ = std::stof(substr);
           if (j % 3 == 0) point.x = value_;
           if (j % 3 == 1) point.y = value_;
           if (j % 3 == 2) point.z = value_;
           j += 1;
        }

        points->push_back(point);
    }

    cloud.setInputCloud(points);
    std::cout << "Number Of Points Loaded: " << points->points.size() << std::endl << std::endl;
}

void getIndices(FeatureCloud &cloud)
{
	const PointCloud::Ptr cloudin_ptr = cloud.getPointCloud();
	const PointCloud::Ptr keypoints_cal_ptr = cloud.getKeyPoints();
	const PointCloud::Ptr keypoints_real_ptr (new PointCloud);
	pcl::PointIndices::Ptr indices_ptr (new pcl::PointIndices);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloudin_ptr);
	std::vector<float>pointNKNSquareDistance;
	std::vector<int> pointIdxNKNSearch;

	int indice = 0;
	for (size_t i =0; i < keypoints_cal_ptr->size();i++)
	{
		kdtree.nearestKSearch(keypoints_cal_ptr->points[i],1,pointIdxNKNSearch,pointNKNSquareDistance);

		indice = pointIdxNKNSearch[0];
		indices_ptr->indices.push_back(indice);
		keypoints_real_ptr->points.push_back(cloudin_ptr->points[indice]);
	}

	cloud.setKeyPoints(keypoints_real_ptr);
	cloud.setKeyPoint_indices(indices_ptr);
}

void writeMatrix(std::ofstream &stream_name, std::string file_name, Eigen::Matrix4f &stream_data)
{
	stream_name.open (file_name, std::fstream::app);

	for(int i = 0; i < 3; i ++)
	{
		for(int j = 0; j < 4; j ++)
		{
			stream_name << ("%lf", stream_data(i,j));
			if (j != 4)
				stream_name << " ";
		}
	}
	stream_name << "\n";
	stream_name.close();
}

void printMatrix(std::string matrix_name, Eigen::Matrix4f &matrix)
{
	std::cout << matrix_name << ": " << std::endl;
	for(int i = 0; i < 3; i ++)
	{
		for(int j = 0; j < 4; j ++)
		{
			std::cout << ("%lf", matrix(i,j));
			if (j != 4)
				std::cout << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

void detectKeyPoints(FeatureCloud &cloud)
{
	if (g_KEYPOINTS_MODULE == "NARF")
	{
		std::cout << "NARF Keypoints Are Being Detected!" << std::endl;
		keyPointsNARF(cloud);
	}

	if (g_KEYPOINTS_MODULE == "SIFT")
	{
		std::cout << "SIFT Keypoints Are Being Detected!" << std::endl;
		keyPointsSIFT(cloud);
	}

	if (g_KEYPOINTS_MODULE == "HARRIS")
	{
		std::cout << "HARRIS Keypoints Are Being Detected!" << std::endl;
		keyPointsHARRIS(cloud);
	}	

	getIndices(cloud);
}

void describeFeatures(FeatureCloud &cloud)
{
	if (g_FEATURE_MODULE == "FPFH")
	{
		std::cout << "FPFH Descriptors Are Being Detected!" << std::endl;
		computeFeatures_FPFH (cloud, g_FEATURE_SEARCH_RADIUS);
	}

	if (g_FEATURE_MODULE == "SHOT")
	{
		std::cout << "SHOT Descriptors Are Being Detected!" << std::endl;
		computeFeatures_SHOT (cloud, g_FEATURE_SEARCH_RADIUS);
	}
}

void removeNANFromNormal(FeatureCloud &cloud)
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

	std::cout << "Remove NAN From Normals:" << std::endl;
	std::cout << (num_normal - normals_->size()) << " Normals Are Removed: " \
	<< num_normal << "->" << normals_->size() << std::endl;
	std::cout << (num_point - nanremoved_->size()) << " Points Are Removed: " \
	<< num_point << "->" << nanremoved_->size() << std::endl << std::endl;

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

	std::cout << "Remove NAN From Feature Descriptors:" << std::endl;
	std::cout << (feature_descriptor->points.size() - nanremoved->points.size()) << " Feature Descriptors Are Removed: " \
	<< feature_descriptor->points.size() << "->" << nanremoved->points.size() << std::endl << std::endl;
}
