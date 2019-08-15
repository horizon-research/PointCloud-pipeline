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

#ifndef PCL_FEATURES_IMPL_NORMAL_3D_H_
#define PCL_FEATURES_IMPL_NORMAL_3D_H_

#include <pcl/features/normal_3d.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::NormalEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  /** If using the Customized KD-Tree data structure **/
  if (use_customized_tree_)
  {
    points_target_.clear();
    for(size_t i = 0; i < surface_->size(); i++)
    {
      // points
      Point point;
      point.x = (*surface_)[i].data[0];
      point.y = (*surface_)[i].data[1];
      point.z = (*surface_)[i].data[2];
      points_target_.push_back(point);
    }
    points_target_ptr_ = &points_target_;

    // indices
    index_target_.clear();
    for(size_t i = 0; i < surface_->size(); i++)
      index_target_.push_back(i);
    index_target_ptr_ = &index_target_;

    buildHKdTree_target();

    std::vector<int> nn_indices (0);
    std::vector<float> nn_dists (0);

    int search_state = 0;

    output.is_dense = true;
    // Save a few cycles by not checking every point for\
    // NaN/Inf values if the cloud is set to dense
    if (input_->is_dense)
    {
      // Iterating over the entire index vector
      for (size_t idx = 0; idx < indices_->size (); ++idx)
      {
        // flann's 
        // search_state = this->searchForNeighbors \
        ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);
        Point query;
        query.x = (input_->points[ (*indices_)[idx] ]).data[0];
        query.y = (input_->points[ (*indices_)[idx] ]).data[1];
        query.z = (input_->points[ (*indices_)[idx] ]).data[2];

        // !Remember to add resize(0) or clear() 
        nn_indices.resize(0);
        nn_dists.resize(0);

        treeH_target_->radiusSearchApprox(query, \
          search_parameter_, nn_indices, nn_dists);

        if (nn_indices.size() == 0)
        {
          output.points[idx].normal[0] = \
          output.points[idx].normal[1] = \
          output.points[idx].normal[2] = \
          output.points[idx].curvature = \
          std::numeric_limits<float>::quiet_NaN ();
          output.is_dense = false;
          continue;
        }

        computePointNormal (*surface_, nn_indices,
                            output.points[idx].normal[0], \
                            output.points[idx].normal[1], \
                            output.points[idx].normal[2], \
                            output.points[idx].curvature);

        flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], \
          vpx_, vpy_, vpz_,
          output.points[idx].normal[0], \
          output.points[idx].normal[1], \
          output.points[idx].normal[2]);

      }
    }
    else
    {
      // Iterating over the entire index vector
      for (size_t idx = 0; idx < indices_->size (); ++idx)
      {
        // flann's 
        // search_state = this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);
        Point query;
        query.x = (input_->points[ (*indices_)[idx] ]).data[0];
        query.y = (input_->points[ (*indices_)[idx] ]).data[1];
        query.z = (input_->points[ (*indices_)[idx] ]).data[2];

        nn_indices.resize(0);
        nn_dists.resize(0);

        treeH_target_->radiusSearchApprox(query, search_parameter_, nn_indices, nn_dists);

        if (!isFinite ((*input_)[(*indices_)[idx]]) || nn_indices.size() == 0 ) //search_state == 0)
        {
          output.points[idx].normal[0] = \
          output.points[idx].normal[1] = \
          output.points[idx].normal[2] = \
          output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

          output.is_dense = false;
          continue;
        }

        computePointNormal (*surface_, nn_indices,
                            output.points[idx].normal[0], \
                            output.points[idx].normal[1], \
                            output.points[idx].normal[2], \
                            output.points[idx].curvature);

        flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                    output.points[idx].normal[0], \
                                    output.points[idx].normal[1], \
                                    output.points[idx].normal[2]);

      }
    }
    if (save_approx_data_)
    {
      approx_radius_ops_num_ = treeH_target_->approx_radius_ops_num_;
      approx_leaders_num_ = treeH_target_->approx_leaders_num_;
      approx_followers_num_ = treeH_target_->approx_followers_num_;
    }
  }

  // Use Flann's implementation
  else
  {
    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices (k_);
    std::vector<float> nn_dists (k_);

    int search_state = 0;
    output.is_dense = true;
    // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
    if (input_->is_dense)
    {
      // Iterating over the entire index vector
      for (size_t idx = 0; idx < indices_->size (); ++idx)
      {
        // flann's 
        search_state = this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);
        
        if (search_state == 0)
        {
          output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();
          output.is_dense = false;
          continue;
        }

        computePointNormal (*surface_, nn_indices,
                            output.points[idx].normal[0], \
                            output.points[idx].normal[1], \
                            output.points[idx].normal[2], \
                            output.points[idx].curvature);

        flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                    output.points[idx].normal[0], \
                                    output.points[idx].normal[1], \
                                    output.points[idx].normal[2]);
      }
    }
    else
    {
      // Iterating over the entire index vector
      for (size_t idx = 0; idx < indices_->size (); ++idx)
      {
        // flann's 
        search_state = this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

        if (search_state == 0)
        {
          output.points[idx].normal[0] = \
          output.points[idx].normal[1] = \
          output.points[idx].normal[2] = \
          output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();
          output.is_dense = false;
          continue;
        }

        computePointNormal (*surface_, nn_indices,
                            output.points[idx].normal[0], \
                            output.points[idx].normal[1], \
                            output.points[idx].normal[2], \
                            output.points[idx].curvature);

        flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                    output.points[idx].normal[0], \
                                    output.points[idx].normal[1], \
                                    output.points[idx].normal[2]);
      }
    }
  }
}

#define PCL_INSTANTIATE_NormalEstimation(T,NT) template class PCL_EXPORTS pcl::NormalEstimation<T,NT>;

#endif    // PCL_FEATURES_IMPL_NORMAL_3D_H_ 
