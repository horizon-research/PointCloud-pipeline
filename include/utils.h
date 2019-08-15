#ifndef UTILS_H
#define UTILS_H

#include "typedefs.h"

extern "C" {

void readConfigFile(std::string file_name);
void makeResultFolder(std::string &result_dir);
	
void detectKeyPoints(FeatureCloud &cloud);
void describeFeatures(FeatureCloud &cloud);

void removeNANFromNormal(FeatureCloud &cloud);
void removeNANFromFPFH(FPFH_Features::Ptr descriptor, FPFH_Features::Ptr nanremoved, FeatureCloud &cloud);

void keyPointsNARF(FeatureCloud &cloud);
void keyPointsSIFT(FeatureCloud &cloud);
void keyPointsHARRIS(FeatureCloud &cloud);

void getIndices (FeatureCloud &cloud);

bool sortString (std::string i,std::string j);
void listDir(std::string path_dir, std::vector<std::string> &files);
void loadBINFile(std::string infile, FeatureCloud &cloud);
void loadTXTFile(std::string infile, FeatureCloud &cloud);
void writeMatrix(std::ofstream &stream_name, std::string file_name, Eigen::Matrix4f &stream_data);
void printMatrix(std::string matrix_name, Eigen::Matrix4f &matrix);

}

template <typename T>
void writeVector(std::ofstream &stream_name, std::string file_name, std::vector<T> &stream_data)
{
	stream_name.open (file_name, std::fstream::app);

	for(int i = 0; i < stream_data.size(); i ++)
	{
		stream_name << ("%lf", stream_data[i]);
		stream_name << " ";
	}
	stream_name << "\n";
	stream_name.close();
	stream_data.clear();
}

template <class Feature_Descriptor>
void removeNANFromDescriptor(Feature_Descriptor& feature_descriptor, Feature_Descriptor& nanremoved, FeatureCloud &cloud)
{
	PointCloud::Ptr keyPoints_ptr_nanremoved(new PointCloud);
	pcl::PointIndices::Ptr indices_ptr_nanremoved(new pcl::PointIndices);

	for (int i=0; i<feature_descriptor->points.size();i++)
	{

		float p = feature_descriptor->points[i].descriptor[0];
		if (p != p) 
		{
			continue;
		}
		else
		{
			nanremoved->push_back(feature_descriptor->points[i]);
			keyPoints_ptr_nanremoved->push_back(cloud.getKeyPoints()->points[i]);
			indices_ptr_nanremoved->indices.push_back((cloud.getKeyPoint_indices())->indices[i]);
		}
	}

	if(feature_descriptor->points.size() != nanremoved->points.size())
	{
		cloud.setKeyPoints(keyPoints_ptr_nanremoved);
		cloud.setKeyPoint_indices(indices_ptr_nanremoved);
	}

	std::cout << "Remove NAN From Feature Descriptors:" << std::endl;
	std::cout << (feature_descriptor->points.size() - nanremoved->points.size()) << " Feature Descriptors Are Removed: " \
	<< feature_descriptor->points.size() << "->" << nanremoved->points.size() << std::endl << std::endl;
}

#endif