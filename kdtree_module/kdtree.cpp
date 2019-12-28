#include "utils.h"

int main(int argc, char const *argv[])
{
	std::string dataset_dir = "./sample/";
	std::string data_reference = dataset_dir + "000000.bin";
	std::string data_query = dataset_dir + "000001.bin";

	struct timespec start, finish;
	float elapsed_secs;

	std::vector<Point> points_reference;
	std::vector<Point> * points_reference_ptr = &points_reference;
	std::vector<Point> points_query;
	std::vector<Point> * points_query_ptr = &points_query;

	std::vector<int> point_indices;
	std::vector<int> * point_indices_ptr = &point_indices;

	// load points
	load_bin(data_reference, points_reference);
	std::cout << "Number of Points Loaded: " << points_reference.size() << std::endl;

	load_bin(data_query, points_query);
	std::cout << "Number of Points Loaded: " << points_query.size() << std::endl;

	// Initialize indices
	for (int i = 0; i < points_reference.size(); ++i) {
		point_indices.push_back(i);
	}

	int query_num = points_query.size();
	int id_start = 0;
	std::cout << "query number: " << query_num << std::endl << std::endl;

	std::vector< std::vector<int> > k_indices;
	std::vector< std::vector<float> > k_dists;
	k_indices.resize(query_num);
	k_dists.resize(query_num);

	// kdtree
	int max_leaf_size = 32;
	KdTreeH kdt(points_reference_ptr, point_indices_ptr, max_leaf_size, 0.75, 1);

	// build tree
	clock_gettime(CLOCK_MONOTONIC, &start);
	kdt.buildKDTree();
	clock_gettime(CLOCK_MONOTONIC, &finish);
	elapsed_secs = (finish.tv_sec - start.tv_sec);
	elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
	std::cout << "Building Tree: " << elapsed_secs << std::endl;
	std::cout << "leaf node number: " << kdt.get_leaf_number() << std::endl;

	/* kdtree nn search */
	std::vector< std::vector<int> > k_indices_kdt;
	std::vector< std::vector<float> > k_dists_kdt;
	k_indices_kdt.resize(query_num);
	k_dists_kdt.resize(query_num);

	int back_track = 0;

	// kdtree radius search
	std::vector< std::vector<int> > k_indices_radius;
	std::vector< std::vector<float> > k_dists_radius;
	k_indices_radius.resize(query_num);
	k_dists_radius.resize(query_num);

	float search_radius = 0.75;
	back_track = 0;

	/* Darkroom shift experiments */
	// Initialize time - data (intermediate data output)
	// key-value pairs
	std::unordered_map<int, int> data_timeStamp_map;

	// transform order information into time-data pairs
	for (int iii = 0; iii < point_indices.size(); ++iii) {
		data_timeStamp_map[ point_indices[iii] ] = iii;
	}

	// Neighboring Search
	clock_gettime(CLOCK_MONOTONIC, &start);

	for (int i = 0; i < query_num; i++) {
		k_indices_radius[i].resize(0);
		k_dists_radius[i].resize(0);

		Point query = points_query[ point_indices[i + id_start] ];
		back_track += kdt.radiusSearch(query, search_radius, \
			k_indices_radius[i], k_dists_radius[i]);
	}
	
	// std::cout << "backtrack: " << back_track << std::endl;
	clock_gettime(CLOCK_MONOTONIC, &finish);
	elapsed_secs = (finish.tv_sec - start.tv_sec);
	elapsed_secs += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
	std::cout << "KDTree Radius: " << elapsed_secs << std::endl << std::endl;

	return 0;
}
