// #define RECORDING

/****** 
	Data structures 
******/
struct Interval {
	float low, high;
};

/******
	Point structure, to represent individual points 
	in the Point Cloud.
 ******/
struct Point {
public:
	float x;
	float y;
	float z;

	int dim; // dimension

	// to-do: add normal information
	Point () : \
	x(.0), y(.0), z(.0), dim(3) {}
public:
	float operator[] (int i) {
		if (i == 0) return x;
		if (i == 1) return y;
		if (i == 2) return z;

		// Default: return z value 
		return z;
	}
};

/******
 ******/
struct Leader
{
	Point p;
	std::vector<int> neighbors;

	Leader ()
	{
		neighbors.resize(0);
	}
};

/******
 ******/
struct Node 
{
	int s_dim; // split dim
	float s_val; // split value
	
	int leaf_idx;

	Node * left_child;
	Node * right_child;

	/********************** 
	   Bounding box: 
	   includes xmin, xmax, ymin, ymax, zmin, zmax 
	   flann impl: 
	   https://github.com/mariusmuja/flann/blob/master/src/cpp/flann/algorithms/kdtree_single_index.h
	***********************/
	std::vector<Interval> bbox;
	std::vector<int> index_list;

	std::vector<Leader> leaders;

	Node () : \
	s_dim(-1), s_val(0.0), leaf_idx(-1), \
	left_child(NULL), right_child(NULL) 
	{
		bbox.resize(0);
		index_list.resize(0);
		
		leaders.resize(0);
	}

	~Node()
	{
		bbox.clear();
		leaders.clear();
	}
};

/*****************
 	customized KD-Tree data structure 
 ****************/
class KdTreeH
{
public:
	typedef std::vector<Point> Points;
	typedef std::vector<Point> * PointsPtr;
	
	typedef std::vector<int> * IndexPtr;
	typedef Node * NodePtr;

	int approx_nn_ops_num_;
	int approx_radius_ops_num_;
	int approx_leaders_num_;
	int approx_followers_num_;

	KdTreeH(std::vector<Point> * points_ptr, \
		std::vector<int> * index_ptr, \
		int max_leaf_size, float para_Radius, float para_NN): \
		dim_(3), leaf_node_num_(0)
	{
		// Initialization
		index_ = index_ptr; 	// POINTER to indices
		points_ = points_ptr;   // POINTER to points

		max_leaf_size_ = max_leaf_size;
		dim_ = (*points_ptr)[0].dim;	// dimension

		para_Radius_ = para_Radius;
		para_NN_ = para_NN;

		approx_nn_ops_num_ = 0;
		approx_radius_ops_num_ = 0;
		approx_leaders_num_ = 0;
		approx_followers_num_ = 0;
	}

	~KdTreeH() {	deleteNode(root_node_); }

	/******* 
	*******/
	void deleteNode(Node * node_p)
	{
		if (node_p->left_child)		deleteNode(node_p->left_child);
		if (node_p->right_child)	deleteNode(node_p->right_child);
		delete node_p;
	}

	/******* 
	*******/
	void buildKDTree()
	{
		// compute bounding box of the root node
		std::vector<Interval> bbox;
		std::vector<Interval> *bbox_ptr = NULL;

		int left = 0;
		int right = (index_->size()) - 1;

		computeBoundingBox(left, right, bbox);
		bbox_ptr = &bbox;
		root_node_ = divideTree(left, right, bbox_ptr);
	}

	NodePtr get_root() {	return root_node_; }
	int get_leaf_number() {	return leaf_node_num_;}

	/****** 
		Traverse top down to find the leaf node. 
	******/
	NodePtr traverse_tree(NodePtr cur_node, Point query, \
		std::vector<Node *> & backtrack_stack)
	{
		/* Traverse the tree top-down to find the leaf node. */
		while(cur_node->s_dim != -1) // non-leaf node
		{
			int s_dim = cur_node->s_dim;
			float s_val = cur_node->s_val;

			if (query[s_dim] > s_val)
			{
				backtrack_stack.push_back(cur_node->left_child);
				cur_node = cur_node->right_child;
			}
			else
			{
				backtrack_stack.push_back(cur_node->right_child);
				cur_node = cur_node->left_child;
			}
		}
		return cur_node;
	}

	/******* 
		reference for impl:
		https://stackoverflow.com/questions/4578967/cube-sphere-intersection-test
	*******/
	bool check_intersection(Point query, float radius, NodePtr cur_node)
	{
		float dist_squared = radius * radius;

		float x_min = (cur_node->bbox)[0].low;
		float x_max = (cur_node->bbox)[0].high;

		float y_min = (cur_node->bbox)[1].low;
		float y_max = (cur_node->bbox)[1].high;
		
		float z_min = (cur_node->bbox)[2].low;
		float z_max = (cur_node->bbox)[2].high;

		if (query.x < x_min)
			dist_squared -= (query.x - x_min) * (query.x - x_min);
		else if (query.x > x_max)
			dist_squared -= (query.x - x_max) * (query.x - x_max);

		if (query.y < y_min)
			dist_squared -= (query.y - y_min) * (query.y - y_min);
		else if (query.y > y_max)
			dist_squared -= (query.y - y_max) * (query.y - y_max);

		if (query.z < z_min)
			dist_squared -= (query.z - z_min) * (query.z - z_min);
		else if (query.z > z_max)
			dist_squared -= (query.z - z_max) * (query.z - z_max);

		return dist_squared > 0;
	}

	/******
		Search (radius) inside a node (not necessarily a leaf node).
	******/
	void radiusSearch_unit(std::vector<int> ind, \
		Point query, float radius, \
		std::vector<int> &k_indices, \
		std::vector<float> &k_dists)
	{
		float dist_ = 0.0;
		for (int i = 0; i < ind.size(); i++)
		{
			dist_ = dist(query, (*points_)[ ind[i] ]);
#ifdef RECORDING
			approx_radius_ops_num_ += 1;
#endif
			if (dist_ < radius)
			{
				k_indices.push_back(ind[i]);
				k_dists.push_back(dist_);
			}
		}
	}

	/******
		Radius Search
	******/
	int radiusSearch(Point query, float radius, \
		std::vector<int> &k_indices, \
		std::vector<float> &k_dists)
	{
		int back_track = 0;
		std::vector<NodePtr> backtrack_stack;
		
		NodePtr cur_node = root_node_;
		cur_node = traverse_tree(cur_node, query, backtrack_stack);

		/* Search in the leaf node. */
		radiusSearch_unit(cur_node->index_list, query, radius, \
			k_indices, k_dists);

		/* Backtrack && prune  */
		for(;backtrack_stack.size();)
		{
			cur_node = backtrack_stack.back();
			backtrack_stack.pop_back();
			
			float dist_min_ = radius;

			// Prune
			bool visit_ = check_intersection(query, dist_min_, cur_node);
			if (visit_)
			{
				back_track += 1;
				if (cur_node->s_dim != -1)
					cur_node = traverse_tree(cur_node, \
						query, backtrack_stack);
				radiusSearch_unit(cur_node->index_list, query, \
					radius, k_indices, k_dists);
			}	
		}

		return back_track;
	}

	/******
		Searching Unit.

		Search inside a node in Brute Force manner.
		(The node is not necessarily a leaf node, since 
		in our implementation each node maintains a list
		of indices.)
	*******/
	void bruteForceSearch(std::vector<int> ind, Point query, \
		std::vector<int> &k_indices, \
		std::vector<float> &k_dists)
	{
		float dist_ = 0.0;

		for (int i = 0; i < ind.size(); i++)
		{
			dist_ = dist(query, (*points_)[ ind[i] ]);

#ifdef RECORDING
			approx_nn_ops_num_ += 1;
#endif

			// caution:
			// make sure to k_dists.resize(0) and k_indices.resize(0)
			// initially, otherwise the code below doesn't work
			if (k_dists.size())
			{
				if (dist_ < k_dists[0])
				{
					k_indices.pop_back();
					k_dists.pop_back();

					k_indices.push_back(ind[i]);
					k_dists.push_back(dist_);
				}
			}
			else
			{
				k_indices.push_back(ind[i]);
				k_dists.push_back(dist_);
			}
		}
	}

	/******
		Nearest Neighbor Search
	******/
	int nearestKSearch(Point query, int knn, \
		std::vector<int> &k_indices, \
		std::vector<float> &k_dists)
	{
		int back_track = 0;
		std::vector<NodePtr> backtrack_stack;
		
		NodePtr cur_node = root_node_;
		cur_node = traverse_tree(cur_node, query, backtrack_stack);

		/* Search the leaf node. */
		bruteForceSearch(cur_node->index_list, query, k_indices, k_dists);

		/* Backtrack && prune  */
		for(;backtrack_stack.size();)
		{
			cur_node = backtrack_stack.back();
			backtrack_stack.pop_back();

			float dist_min_ = k_dists[0]; // radius

			// Prune
			bool visit_ = check_intersection(query, dist_min_, cur_node);
			if (visit_)
			{
				back_track += 1;
				if (cur_node->s_dim != -1)
					cur_node = traverse_tree(cur_node, query, backtrack_stack);
				bruteForceSearch(cur_node->index_list, query, k_indices, k_dists);
			}	
		}
		return back_track;
	}

	/******
	******/
	bool bruteForceSearchUnitApprox(NodePtr cur_leaf, Point query_point, std::vector<int> &k_indices, std::vector<float> &k_dists)
	{
		bool findleader = false;

		if ((cur_leaf->leaders).size())
		{
			std::vector<float> dis_list;
			for (int i = 0; i < (cur_leaf->leaders).size(); i++)
			{
				float dis = getDistance(query_point, (cur_leaf->leaders)[i].p);
				dis_list.push_back(dis);
			}

			int index = min_element(dis_list.begin(), dis_list.end()) - dis_list.begin();
			float dis_min = sqrt(dis_list[index]);

			if (dis_min < para_NN_)
			{
				bruteForceSearch(((cur_leaf->leaders)[index].neighbors), query_point, k_indices, k_dists);
				findleader = true;
#ifdef RECORDING
				approx_followers_num_ += 1;
#endif
			}
			else
			{
				bruteForceSearch(cur_leaf->index_list, query_point, k_indices, k_dists);
#ifdef RECORDING
				approx_leaders_num_ += 1;
#endif
			}
		}
		else
		{
			bruteForceSearch(cur_leaf->index_list, query_point, k_indices, k_dists);
#ifdef RECORDING
			approx_leaders_num_ += 1;
#endif
		}
		return findleader;
	}

	/******
	******/
	int nearestKSearchApprox(Point query, int knn, \
		std::vector<int> &k_indices, \
		std::vector<float> &k_dists)
	{
		int back_track = 0;
		std::vector<NodePtr> backtrack_stack;
		
		NodePtr cur_node = root_node_;
		cur_node = traverse_tree(cur_node, query, backtrack_stack);

		/* Search the leaf node. */
		bool findleader = bruteForceSearchUnitApprox(cur_node, query, k_indices, k_dists);

		// Backtrack && prune 
		if (!findleader)
		{
			for(;backtrack_stack.size();)
			{
				NodePtr backtrack_node = backtrack_stack.back();
				backtrack_stack.pop_back();

				float dist_min_ = k_dists[0]; // radius

				// Prune
				bool visit_ = check_intersection(query, dist_min_, backtrack_node);
				if (visit_)
				{
					back_track += 1;
					if (backtrack_node->s_dim != -1)
						backtrack_node = traverse_tree(backtrack_node, query, backtrack_stack);

					bruteForceSearch(backtrack_node->index_list, query, k_indices, k_dists);
				}
			}
			Leader leader_point;
			leader_point.p = query;
			(leader_point.neighbors).push_back((k_indices[0]));
			(cur_node->leaders).push_back(leader_point);
		}
		return back_track;
	}

	/******
	******/
	bool radiusSearchUnitApprox(NodePtr cur_leaf, Point query_point, float radius, std::vector<int> &k_indices, std::vector<float> &k_dists)
	{
		bool findleader = false;

		// if leader already exist on leaf node
		if ((cur_leaf->leaders).size())
		{
			std::vector<float> dis_list;
			for (int i = 0; i < (cur_leaf->leaders).size(); i++)
			{
				float dis = getDistance(query_point, (cur_leaf->leaders)[i].p);
				dis_list.push_back(dis);
			}

			int index = min_element(dis_list.begin(), dis_list.end()) - dis_list.begin();
			float dis_min = sqrt(dis_list[index]);

			if (dis_min < para_Radius_ * radius)
			{
				radiusSearch_unit(((cur_leaf->leaders)[index].neighbors), query_point, radius, k_indices, k_dists);
				findleader = true;
#ifdef RECORDING
				approx_followers_num_ += 1;
#endif			
			}
			else
			{
				radiusSearch_unit(cur_leaf->index_list, query_point, radius, k_indices, k_dists);
#ifdef RECORDING
				approx_leaders_num_ += 1;
#endif
			}
		}
		// create new leader
		else
		{
			radiusSearch_unit(cur_leaf->index_list, query_point, radius, k_indices, k_dists);
#ifdef RECORDING
			approx_leaders_num_ += 1;
#endif		
		}

		return findleader;
	}

	/******
	******/
	int radiusSearchApprox(Point query, float radius, \
		std::vector<int> &k_indices, \
		std::vector<float> &k_dists)
	{
		int back_track = 0;
		std::vector<NodePtr> backtrack_stack;
		
		NodePtr cur_node = root_node_;
		cur_node = traverse_tree(cur_node, query, backtrack_stack);

		/* Search the leaf node. */
		bool findleader = radiusSearchUnitApprox(cur_node, query, radius, k_indices, k_dists);

		// Backtrack && prune 
		if (!findleader)
		{
			for(;backtrack_stack.size();)
			{
				NodePtr backtrack_node = backtrack_stack.back();
				backtrack_stack.pop_back();

				float dist_min_ = radius;

				// Prune
				bool visit_ = check_intersection(query, dist_min_, backtrack_node);
				if (visit_)
				{
					back_track += 1;
					if (backtrack_node->s_dim != -1)
						backtrack_node = traverse_tree(backtrack_node, query, backtrack_stack);
					radiusSearch_unit(backtrack_node->index_list, query, radius, k_indices, k_dists);
				}	
			}
			Leader leader_point;
			if (k_indices.size())
			{
				leader_point.p = query;
				for (int i = 0; i < k_indices.size(); i++)
					(leader_point.neighbors).push_back((k_indices[i]));
				(cur_node->leaders).push_back(leader_point);
			}
		}
		return back_track;
	}

private:

	NodePtr root_node_;

	IndexPtr index_;
	PointsPtr points_;

	int dim_; 
	// point data dimension (by default 3)

	int max_leaf_size_; 
	// to control the leaf node size

	int leaf_node_num_;

	std::vector<NodePtr> leaf_nodes_;
	// store the pointers to a leaf node
	// so that we could get the content of the leaf node by index

	// Parameter for Approx radius search
	float para_Radius_;

	// Parameter for Approx NN search
	float para_NN_;

private:
	float getDistance(Point point, Point centroid)
	{
		float distance = pow((centroid.x - point.x), 2) + pow((centroid.y - point.y), 2) + pow((centroid.z - point.z), 2);
		return distance;
	}

	/*
		Helper function
	*/
	float dist(Point p1, Point p2)
	{
		return sqrt(pow((p1.x - p2.x),2) + \
			pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));
	}

	/******
		Divide the tree.
		The tree is built with this function in a 
		recursive fashion.

		Args:
		Returns:
	******/
	NodePtr divideTree(int left, int right, std::vector<Interval> *bbox_ptr)
	{
		NodePtr node = new Node();

		// find the indices for the current node
		for (int i = left; i <= right; ++i) {
			(node->index_list).push_back((*index_)[i]);
		}
		// store the bounding box
		node->bbox = *bbox_ptr;

		int count = right - left;

		if(count <= max_leaf_size_)
		{
			// leaf node
			node->s_dim = -1;
			node->s_val = -1;

			node->leaf_idx = leaf_node_num_;
			leaf_node_num_ += 1;

			leaf_nodes_.push_back(node);

			return node;
		}
		else
		{
			// splitting dim
			int split_dim = 0;
			float span = 0.0;
			std::vector<float> value_list;

			findSplitDim(split_dim, span, bbox_ptr);
			node->s_dim = split_dim;

			// splitting value
			float split_val = 0.0;

			// Original Implementation by QuickSort and Select Median Value
			// findMedian(left, right, split_dim, split_val);

			//Alternative Version to Select Median by QuickSelect
			getValueList(left, right, split_dim, value_list);
			std::vector<float>* value_list_ptr = &value_list;
			qSelectMedian(value_list_ptr, split_val);

			// FLANN's implementation:
			// split_val = ((*bbox_ptr)[split_dim].low + 
			// (*bbox_ptr)[split_dim].high) / 2;
			node->s_val = split_val;

			// process the indices
			int lim1 = 0, lim2 = 0, split_delta = 0;
			planeSplit(left, right, split_dim, split_val, lim1, lim2);
			split_delta = (lim1 + lim2) / 2;

			std::vector<Interval> bbox_l;
			std::vector<Interval> bbox_r;
			computeBoundingBox(left, left + lim2, bbox_l);
			computeBoundingBox(left + lim2 + 1, right, bbox_r);

			node->left_child = divideTree(left, left + lim2, &bbox_l);
			node->right_child = divideTree(left + lim2 + 1, right, &bbox_r);

			return node;
		}
	}

	/**********************************************************************
		Using flann's implementation to find the splitting index.
		The idea is to split the points into three parts by changing the 
		order of the indices. By the end of planeSplit, the index_ variable 
		is re-arranged:

			index_ [left : lim1]: 
				indices of points whose value on the split dim < spit value

			index_ [lim1 : lim2]: 
				indices of points whose value on the split dim == spit value

			index_ [lim2 : right]: 
				indices of points whose value on the split dim > spit value

	**********************************************************************/
	void planeSplit(int left, int right, int split_dim, 
		float split_val, int& lim1, int& lim2)
	{
		int start = 0;
		int end = right - left;

		for (;;) {
			while (start <= end && (*points_)[(*index_)[left + start]][split_dim] < split_val) 
				++start;
			while (start <= end && (*points_)[(*index_)[left + end]][split_dim] >= split_val) 
				--end;
			
			if (start > end) break;
			std::swap((*index_)[left + start], (*index_)[left + end]); ++start; --end;
		}
		lim1 = start;

		end = right - left;
		for (;; ) {
			while (start <= end && (*points_)[(*index_)[left + start]][split_dim] <= split_val) 
				++start;
			while (start <= end && (*points_)[(*index_)[left + end]][split_dim] > split_val) 
				--end;
			if (start > end) break;
			std::swap((*index_)[left + start], (*index_)[left + end]); ++start; --end;
		}
		lim2 = end;
	}

	/******
	******/
	inline void swap(std::vector<float>* value_list, int a, int b) 
	{
		float tmpa = (*value_list)[a];
		float tmpb = (*value_list)[b];
		(*value_list)[a] = tmpb;
		(*value_list)[b] = tmpa;
	}

	/******
	******/
	void getValueList(int left, int right, int split_dim, std::vector<float> &value_list)
	{
		for (int i = left; i <= right; i++)
			value_list.push_back((*points_)[(*index_)[i]][split_dim]);
	}

	/******
	******/
	void qSelectMedian(std::vector<float>* value_list, float &median_value)
	{
		int left = 0;
		int right = value_list->size() - 1;
		int middle = value_list->size() / 2;
		while(1)
		{
			float pivot = (*value_list)[middle];
			swap(value_list, middle, right);
			int store = left;

			for (int i = left; i < right; i++)
			{
				if ((*value_list)[i] < pivot)
				{
					if (i != store)
						swap(value_list, i, store);
					store++;
				}
			}
			swap(value_list, store, right);

			if ((*value_list)[store] == (*value_list)[middle])
			{
				median_value = (*value_list)[middle];
				break;
			}

			if (store > middle)	right = store - 1;
			else				left = store;
		}
	}


	/******
		Calculate the median value
		reference:
		https://stackoverflow.com/questions/10662013/finding-the-median-of-an-unsorted-array
	******/
	void findMedian(int left, int right, int split_dim, float &median_value)
	{
		std::vector<float> dim_sorted;
		for (int i = left; i <= right; ++i)
		{
			dim_sorted.push_back((*points_)[(*index_)[i]][split_dim]);
		}
		std::sort(dim_sorted.begin(), dim_sorted.end());

		int median_pos = (right - left) / 2;

		median_value = dim_sorted[median_pos];
	}

	/******
		Compute the bounding box of current node
		given indices of points. 
	******/
	void computeBoundingBox(int left, int right, \
		std::vector<Interval> & bbox)
	{
		int cur_dim = 0;
		for ( ;cur_dim < dim_; cur_dim++)
		{
			Interval bounds;
			computeMinMax(left, right, cur_dim, bounds.low, bounds.high);
			bbox.push_back(bounds);
		}
	}

	/******
		Choose the split dimension.
		Current strategy: the one with maximum span value.
		span == MAX(dim) - MIN(dim)
	******/
	void findSplitDim(int &best_dim, float &span, \
		std::vector<Interval> *bbox_ptr)
	{
		int cur_dim = 0; // 0: x, 1: y, 2: z
		
		float min_ = 0.0, max_ = 0.0; 
		span = 0.0;
		
		for ( ;cur_dim < dim_; cur_dim++)
		{
			min_ = (*bbox_ptr)[cur_dim].low;
			max_ = (*bbox_ptr)[cur_dim].high;

			if ((max_ - min_) > span)
			{
				best_dim = cur_dim;
				span = (max_ - min_);
			}
		}
	}

	/****** 
		Give specified indices (index_[left] to index_[right]), 
		compute the span (max value - min value) 
		on a specific dimension.
	******/
	void computeMinMax(int left, int right, int dim, float& min_val, float& max_val)
	{
		min_val = (*points_)[(*index_)[left]][dim];
		max_val = (*points_)[(*index_)[left]][dim];

		for (int i = left + 1; i <= right; ++i) {

			float val = (*points_)[(*index_)[i]][dim];
			
			if (val < min_val) min_val = val;
			if (val > max_val) max_val = val;
		}
	}
};
