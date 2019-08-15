/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_

#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::setInputCloud (const typename pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::PointCloudSourceConstPtr &cloud)
{
  // PCL_ERROR("setInputCloud\n");
  setInputSource (cloud); 
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> typename pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::PointCloudSourceConstPtr const
pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getInputCloud ()
{
  return (getInputSource ()); 
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::setInputTarget (
    const PointCloudTargetConstPtr &cloud)
{
  if (cloud->points.empty ())
  {
    PCL_ERROR ("[pcl::registration::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }
  target_ = cloud;

  // Set the internal point representation of choice
  if (point_representation_)
    tree_->setPointRepresentation (point_representation_);

  target_cloud_updated_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute ()
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  // Only update target kd-tree if a new target cloud was set
  if (target_cloud_updated_ && !force_no_recompute_)
  {
    if(!use_customized_tree_)
    { /** Use FLANN **/
      if (target_indices_) tree_->setInputCloud (target_, target_indices_);
      else tree_->setInputCloud (target_);

      target_cloud_updated_ = false;
    }
    else
    { /** Use the customized tree only for 3-d points. **/
      if (std::is_same<PointSource, pcl::PointXYZ>::value ||
          std::is_same<PointSource, pcl::PointNormal>::value)
      {
        /** Initialize the data structure used by the customized tree. **/
        points_target_.clear();
        for(size_t i = 0; i < target_->size(); i++)
        {
          Point point;
          point.x = (*target_)[i].data[0];
          point.y = (*target_)[i].data[1];
          point.z = (*target_)[i].data[2];

          points_target_.push_back(point);
        }
        points_target_ptr_ = &points_target_;

        // Initialize indices
        if(target_indices_)
        {   /** if the target indices are given **/
          index_target_.clear();
          for(size_t i = 0; i < target_indices_->size(); i++)
            index_target_.push_back((*target_indices_)[i]);
        }
        else
        {
          index_target_.clear();
          for(size_t i = 0; i < target_->size(); i++)
            index_target_.push_back(i);
        }
        index_target_ptr_ = &index_target_;
 
        if (treeH_target_)  delete treeH_target_;
        buildHKdTree_target();
      }
      else
      {
        PCL_ERROR ("Customized KD-Tree Only Accept Data with Three Dimensions!\n");
        exit(-1);
      }
    }
  }

  return (PCLBase<PointSource>::initCompute ());
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal ()
{
  // Only update source kd-tree if a new target cloud was set
  if (source_cloud_updated_ && !force_no_recompute_reciprocal_)
  {
    if (!use_customized_tree_)
    {
      if (point_representation_)
        tree_reciprocal_->setPointRepresentation (point_representation_);
      // If the target indices have been given via setIndicesTarget
      if (indices_)
        tree_reciprocal_->setInputCloud (getInputSource(), getIndicesSource());
      else
        tree_reciprocal_->setInputCloud (getInputSource());

      source_cloud_updated_ = false;
    }
    
    else
    {
      // Only for 3-d points.
      if (std::is_same<PointSource, pcl::PointXYZ>::value ||
          std::is_same<PointSource, pcl::PointNormal>::value)
      {
        points_src_.clear();
        for(size_t i = 0; i < input_->size(); i++)
        { 
          Point point;
          point.x = ((*input_)[i]).data[0];
          point.y = ((*input_)[i]).data[1];
          point.z = ((*input_)[i]).data[2];

          points_src_.push_back(point);
        }
        points_src_ptr_ = &points_src_;

        // Initialize indices
        if(indices_)
        {
          index_src_.clear();
          for(size_t i = 0; i < indices_->size(); i++)
            index_src_.push_back((*indices_)[i]);
        }
        else
        {
          index_src_.clear();
          for(size_t i = 0; i < input_->size(); i++)
            index_src_.push_back(i);
        }
        index_src_ptr_ = &index_src_;

        if (treeH_source_)  delete treeH_source_;
        buildHKdTree_source();
      }
      else
      {
        PCL_ERROR ("The customized KD-Tree Only Accept Data with Three Dimensions!\n");
        exit(-1);
      } 
    }
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{
  if (!initCompute ())
    return;

  double max_dist_sqr = max_distance * max_distance;

  correspondences.resize (indices_->size ());

  int Search_K = 1;

  std::vector<int> index (Search_K);
  std::vector<float> distance (Search_K);
  pcl::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;

  int back_track = 0;
  
  // Check if the template types are the same. If true, avoid a copy.
  // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
  if (isSamePointType<PointSource, PointTarget> ())
  {
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); \
      idx != indices_->end (); ++idx)
    {
      if (use_customized_tree_)
      {
        if (std::is_same<PointSource, pcl::PointXYZ>::value ||
            std::is_same<PointSource, pcl::PointNormal>::value)
        {
          // Only for 3-d points
          // reason: currently the implementation does not support 
          // high dimensional tree building and searching

          Point query;
          query.x = (input_->points[*idx]).data[0];
          query.y = (input_->points[*idx]).data[1];
          query.z = (input_->points[*idx]).data[2];

          // !!! Be sure to add these two lines.
          index.resize(0);
          distance.resize(0);

          back_track += treeH_target_->nearestKSearchApprox(query, 1, index, distance);
          // currently, our implementation only support the nearest neighbor search
          // i.e., K == 1
        }
        else
        {
          PCL_ERROR ("The customized KD-Tree Only Accept Data with Three Dimensions!\n");
          exit(-1);
        }
      }
      else
      { /** Use FLANN **/
        tree_->nearestKSearch (input_->points[*idx], Search_K, index, distance);
      }

      if (distance[0] > max_dist_sqr)
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0]; // the nearest neighbor
      corr.distance = distance[0]; // the nearest neighbor
      correspondences[nr_valid_correspondences++] = corr;
    }
    if ((std::is_same<PointSource, pcl::PointXYZ>::value ||
      std::is_same<PointSource, pcl::PointNormal>::value) && save_approx_data_)
    {
      approx_nn_ops_num_ += treeH_target_->approx_nn_ops_num_; 
      approx_leaders_num_ += treeH_target_->approx_leaders_num_;
      approx_followers_num_ += treeH_target_->approx_followers_num_;
    }
  }
  else {}

  correspondences.resize (nr_valid_correspondences);
  deinitCompute ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineReciprocalCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{
  if (!initCompute ())
    return;

  if (!initComputeReciprocal())
    return;

  double max_dist_sqr = max_distance * max_distance;
  correspondences.resize (indices_->size());
  
  int search_K = 1;

  std::vector<int> index (search_K);
  std::vector<float> distance (search_K);
  std::vector<int> index_reciprocal (search_K);
  std::vector<float> distance_reciprocal (search_K);

  pcl::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;

  int target_idx = 0;
  int back_track = 0;

  // Check if the template types are the same. If true, avoid a copy.
  // Both point types MUST be registered using the 
  // POINT_CLOUD_REGISTER_POINT_STRUCT macro!
  if (isSamePointType<PointSource, PointTarget> ())
  { 
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      if (use_customized_tree_)
      {
        if (std::is_same<PointSource, pcl::PointXYZ>::value ||
            std::is_same<PointSource, pcl::PointNormal>::value)
        {
          // Only the customized tree for 3-d points
          Point query;
          query.x = (input_->points[*idx]).data[0];
          query.y = (input_->points[*idx]).data[1];
          query.z = (input_->points[*idx]).data[2];
          
          index.resize(0);
          distance.resize(0);

          back_track += treeH_target_->nearestKSearchApprox(query, search_K, index, distance);
        }
        else
        {
          PCL_ERROR ("The customized KD-Tree Only Accept Data with Three Dimensions!\n");
          exit(-1);
        }
      }
      else
        tree_->nearestKSearch (input_->points[*idx], search_K, index, distance);

      if (distance[0] > max_dist_sqr)
        continue;
      target_idx = index[0];

      //------------------Reciprocal Search -------------------------------
      if (use_customized_tree_)
      {
        // Only for 3-d points
        if (std::is_same<PointSource, pcl::PointXYZ>::value ||
            std::is_same<PointSource, pcl::PointNormal>::value)
        {
          Point query;
          query.x = (target_->points[target_idx]).data[0];
          query.y = (target_->points[target_idx]).data[1];
          query.z = (target_->points[target_idx]).data[2];
          
          index_reciprocal.resize(0);
          distance_reciprocal.resize(0);

          back_track += treeH_source_->nearestKSearchApprox(query, search_K, index_reciprocal, distance_reciprocal);
        }
        else
        {
          PCL_ERROR ("The customized KD-Tree Only Accept Data with Three Dimensions!\n");
          exit(-1);
        }
      }
      else
      {
        tree_reciprocal_->nearestKSearch (target_->points[target_idx], search_K, \
          index_reciprocal, distance_reciprocal);
      }

      // If not reciprocal: throw away
      if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0])
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }

    if ((std::is_same<PointSource, pcl::PointXYZ>::value ||
     std::is_same<PointSource, pcl::PointNormal>::value) && save_approx_data_)
    {
      approx_nn_ops_num_ += treeH_target_->approx_nn_ops_num_;
      approx_nn_ops_num_ += treeH_source_->approx_nn_ops_num_;   
      approx_leaders_num_ += treeH_target_->approx_leaders_num_;
      approx_leaders_num_ += treeH_source_->approx_leaders_num_;
      approx_followers_num_ += treeH_target_->approx_followers_num_;
      approx_followers_num_ += treeH_source_->approx_followers_num_; 
    }
  }
  else { }

  correspondences.resize (nr_valid_correspondences);
  deinitCompute ();
}

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_ */