#include "kdtree.h"
#include <cstdlib>

/* Helper functions */

float dist(Point p1, Point p2)
{
	return sqrt(pow((p1.x - p2.x),2) + \
		pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));
}

/*** 
 *	brute force search, n == 1 
 ***/

void brute_force_search(std::vector<Point> &reference, \
	Point query, std::vector<int> &k_indices, \
	std::vector<float> &k_dists)
{
	int knn = 1;

	k_indices.resize(0);
	k_dists.resize(0);

	float dist_ = 0.0;

	for (int i = 0; i < reference.size(); i++)
	{
		dist_ = dist(query, reference[i]);
		// if (dist_ == 0)
		// 	continue;

		if (k_dists.size())
		{
			if (dist_ < k_dists[0])
			{
				k_indices.pop_back();
				k_dists.pop_back();

				k_indices.push_back(i);
				k_dists.push_back(dist_);
			}
		}
		else
		{
			k_indices.push_back(i);
			k_dists.push_back(dist_);
		}
	}
}



void load_bin(std::string infile, std::vector<Point> & points)
{
	std::cout << "Loading " << infile << std::endl;

	std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);

	if(!input.good()){
		std::cerr << "Could not read file: " << infile << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);

	for (int j=0; input.good() && !input.eof(); j++) {
		
		Point point, point_gt;

		input.read((char *) &point.x, 3 * sizeof(float));
		input.seekg(sizeof(float), std::ios::cur);    

		point_gt.x = -point.y;
		point_gt.y = -point.z;
		point_gt.z = point.x;

		points.push_back(point_gt);
	}
	input.close();
}
