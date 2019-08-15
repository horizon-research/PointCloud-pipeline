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

#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_

#include <string>

#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_macros.h>
#include <pcl/registration/correspondence_types.h>

// added from outside
#include <limits>
#include <fstream>
#include <iostream>
#include <string>

namespace pcl
{
  namespace registration
  {
    /** \brief Abstract @b CorrespondenceEstimationBase class. 
      * All correspondence estimation methods should inherit from this.
      * \author Radu B. Rusu
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class CorrespondenceEstimationBase: public PCLBase<PointSource>
    {
      public:
        typedef boost::shared_ptr<CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > ConstPtr;

        // using PCLBase<PointSource>::initCompute;
        using PCLBase<PointSource>::deinitCompute;
        using PCLBase<PointSource>::input_;
        using PCLBase<PointSource>::indices_;
        using PCLBase<PointSource>::setIndices;

        typedef pcl::search::KdTree<PointTarget> KdTree;
        typedef typename KdTree::Ptr KdTreePtr;

        typedef pcl::search::KdTree<PointSource> KdTreeReciprocal;
        typedef typename KdTree::Ptr KdTreeReciprocalPtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        std::vector<Point> points_target_; // target
        std::vector<Point> * points_target_ptr_; // target 
        std::vector<int> index_target_;
        std::vector<int> * index_target_ptr_;

        std::vector<Point> points_src_; // source
        std::vector<Point> * points_src_ptr_; // source

        std::vector<int> index_src_;
        std::vector<int> * index_src_ptr_;

        KdTreeH * treeH_source_;
        KdTreeH * treeH_target_; 

        /** \brief Empty constructor. */
        CorrespondenceEstimationBase () 
          : corr_name_ ("CorrespondenceEstimationBase")
          , tree_ (new pcl::search::KdTree<PointTarget>)
          , tree_reciprocal_ (new pcl::search::KdTree<PointSource>)
          , target_ ()
          , target_indices_ ()
          , point_representation_ ()
          , input_transformed_ ()
          , input_fields_ ()
          , target_cloud_updated_ (true)
          , source_cloud_updated_ (true)
          , force_no_recompute_ (false)
          , force_no_recompute_reciprocal_ (false)
          , treeH_source_ (NULL)
          , treeH_target_ (NULL)
          , use_customized_tree_ (false)
          , max_leaf_size_ (64)
          , approx_radius_para_(0.0)
          , approx_nn_para_(0.0)
          , approx_nn_ops_num_(0.0)
          , approx_leaders_num_(0)
          , approx_followers_num_(0)
          , save_approx_data_(false)
        {
          // PCL_ERROR ("CorrespondenceEstimationBase Init\n");
        }
      
        /** \brief Empty destructor */
        virtual ~CorrespondenceEstimationBase () 
        { 
        }

        /** Build the customized KDTree (for source point cloud). **/
        void buildHKdTree_source()
        {
          treeH_source_ = new KdTreeH(points_src_ptr_, \
            index_src_ptr_, max_leaf_size_, approx_radius_para_, approx_nn_para_); 
          treeH_source_->buildKDTree();
        }

        /** Build the customized KDTree (for target point cloud). **/
        void buildHKdTree_target()
        {
          treeH_target_ = new KdTreeH(points_target_ptr_, \
            index_target_ptr_, max_leaf_size_, approx_radius_para_, approx_nn_para_);
          treeH_target_->buildKDTree();
        }

        /** \brief Provide a pointer to the input source 
          * (e.g., the point cloud that we want to align to the target)
          *
          * \param[in] cloud the input point cloud source
          */
        PCL_DEPRECATED ("[pcl::registration::CorrespondenceEstimationBase::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.")
        void
        setInputCloud (const PointCloudSourceConstPtr &cloud);

        /** \brief Get a pointer to the input point cloud dataset target. */
        PCL_DEPRECATED ("[pcl::registration::CorrespondenceEstimationBase::getInputCloud] getInputCloud is deprecated. Please use getInputSource instead.")
        PointCloudSourceConstPtr const
        getInputCloud ();

        /** \brief Provide a pointer to the input source 
          * (e.g., the point cloud that we want to align to the target)
          *
          * \param[in] cloud the input point cloud source
          */
        inline void 
        setInputSource (const PointCloudSourceConstPtr &cloud)
        {
          source_cloud_updated_ = true;
          PCLBase<PointSource>::setInputCloud (cloud);
          pcl::getFields (*cloud, input_fields_);
        }

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudSourceConstPtr const 
        getInputSource () 
        { 
          return (input_ ); 
        }

        /** \brief Provide a pointer to the input target 
          * (e.g., the point cloud that we want to align the input source to)
          * \param[in] cloud the input point cloud target
          */
        inline void 
        setInputTarget (const PointCloudTargetConstPtr &cloud);

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudTargetConstPtr const 
        getInputTarget () { return (target_ ); }

        /** \brief See if this rejector requires source normals */
        virtual bool
        requiresSourceNormals () const
        { return (false); }

        /** \brief Abstract method for setting the source normals */
        virtual void
        setSourceNormals (pcl::PCLPointCloud2::ConstPtr /*cloud2*/)
        {
          PCL_WARN ("[pcl::registration::%s::setSourceNormals] This class does not require input source normals", getClassName ().c_str ());
        }
        
        /** \brief See if this rejector requires target normals */
        virtual bool
        requiresTargetNormals () const
        { return (false); }

        /** \brief Abstract method for setting the target normals */
        virtual void
        setTargetNormals (pcl::PCLPointCloud2::ConstPtr /*cloud2*/)
        {
          PCL_WARN ("[pcl::registration::%s::setTargetNormals] This class does not require input target normals", getClassName ().c_str ());
        }

        /** \brief Provide a pointer to the vector of indices that represent the 
          * input source point cloud.
          * \param[in] indices a pointer to the vector of indices 
          */
        inline void
        setIndicesSource (const IndicesPtr &indices)
        {
          setIndices (indices);
        }

        /** \brief Get a pointer to the vector of indices used for the source dataset. */
        inline IndicesPtr const 
        getIndicesSource () { return (indices_); }

        /** \brief Provide a pointer to the vector of indices that represent the input target point cloud.
          * \param[in] indices a pointer to the vector of indices 
          */
        inline void
        setIndicesTarget (const IndicesPtr &indices)
        {
          target_cloud_updated_ = true;
          target_indices_ = indices;
        }

        /** \brief Get a pointer to the vector of indices used for the target dataset. */
        inline IndicesPtr const 
        getIndicesTarget () { return (target_indices_); }

        /** \brief Provide a pointer to the search object used to find correspondences in
          * the target cloud.
          * \param[in] tree a pointer to the spatial search object.
          * \param[in] force_no_recompute If set to true, this tree will NEVER be 
          * recomputed, regardless of calls to setInputTarget. Only use if you are 
          * confident that the tree will be set correctly.
          */
        inline void
        setSearchMethodTarget (const KdTreePtr &tree, 
                               bool force_no_recompute = false) 
        { 
          tree_ = tree; 
          if (force_no_recompute)
          {
            force_no_recompute_ = true;
          }
          // Since we just set a new tree, we need to check for updates
          target_cloud_updated_ = true;
        }

        /** \brief Get a pointer to the search method used to find correspondences in the
          * target cloud. */
        inline KdTreePtr
        getSearchMethodTarget () const
        {
          return (tree_);
        }

        /** \brief Provide a pointer to the search object used to find correspondences in
          * the source cloud (usually used by reciprocal correspondence finding).
          * \param[in] tree a pointer to the spatial search object.
          * \param[in] force_no_recompute If set to true, this tree will NEVER be 
          * recomputed, regardless of calls to setInputSource. Only use if you are 
          * extremely confident that the tree will be set correctly.
          */
        inline void
        setSearchMethodSource (const KdTreeReciprocalPtr &tree, 
                               bool force_no_recompute = false) 
        { 
          tree_reciprocal_ = tree; 
          if ( force_no_recompute )
          {
            force_no_recompute_reciprocal_ = true;
          }
          // Since we just set a new tree, we need to check for updates
          source_cloud_updated_ = true;
        }

        /** \brief Get a pointer to the search method used to find correspondences in the
          * source cloud. */
        inline KdTreeReciprocalPtr
        getSearchMethodSource () const
        {
          return (tree_reciprocal_);
        }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ()) = 0;

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ()) = 0;

        /** \brief Provide a boost shared pointer to the PointRepresentation to be used 
          * when searching for nearest neighbors.
          *
          * \param[in] point_representation the PointRepresentation to be used by the 
          * k-D tree for nearest neighbor search
          */
        inline void
        setPointRepresentation (const PointRepresentationConstPtr &point_representation)
        {
          point_representation_ = point_representation;
        }

        /** \brief Clone and cast to CorrespondenceEstimationBase */
        virtual boost::shared_ptr< CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > clone () const = 0;

        inline void setUseCustomizedKDTree(bool use_customized_tree) { use_customized_tree_ = use_customized_tree;}
        inline void setMaxLeafSize(int size) {max_leaf_size_ = size;}
        inline void setSaveApproxData(bool save) {save_approx_data_ = save;}

        inline void setApproxNNPara(float approx_nn_para) { approx_nn_para_ = approx_nn_para; }
        inline void getApproxNNOpsNum(int &approx_nn_ops_num) { approx_nn_ops_num = approx_nn_ops_num_; }
        inline void getApproxLeadersNum(int &approx_leaders_num){ approx_leaders_num = approx_leaders_num_;}
        inline void getApproxFollowersNum(int &approx_followers_num){ approx_followers_num = approx_followers_num_;}

      protected:
        /** \brief The correspondence estimation method name. */
        std::string corr_name_;

        /** \brief A pointer to the spatial search object used for the target dataset. */
        KdTreePtr tree_;

        /** \brief A pointer to the spatial search object used for the source dataset. */
        KdTreeReciprocalPtr tree_reciprocal_;
        
        /** \brief The input point cloud dataset target. */
        PointCloudTargetConstPtr target_;

        /** \brief The target point cloud dataset indices. */
        IndicesPtr target_indices_;

        /** \brief The point representation used (internal). */
        PointRepresentationConstPtr point_representation_;

        /** \brief The transformed input source point cloud dataset. */
        PointCloudTargetPtr input_transformed_;

        /** \brief The types of input point fields available. */
        std::vector<pcl::PCLPointField> input_fields_;

        /** \brief Abstract class get name method. */
        inline const std::string& 
        getClassName () const { return (corr_name_); }

        /** \brief Internal computation initalization. */
        bool
        initCompute ();
        
        /** \brief Internal computation initalization for reciprocal correspondences. */
        bool
        initComputeReciprocal ();

        /** \brief Variable that stores whether we have a new target cloud, meaning we need to pre-process it again.
         * This way, we avoid rebuilding the kd-tree for the target cloud every time the determineCorrespondences () method
         * is called. */
        bool target_cloud_updated_;
        /** \brief Variable that stores whether we have a new source cloud, meaning we need to pre-process it again.
         * This way, we avoid rebuilding the reciprocal kd-tree for the source cloud every time the determineCorrespondences () method
         * is called. */
        bool source_cloud_updated_;
        /** \brief A flag which, if set, means the tree operating on the target cloud 
         * will never be recomputed*/
        bool force_no_recompute_;
        
        /** \brief A flag which, if set, means the tree operating on the source cloud 
         * will never be recomputed*/
        bool force_no_recompute_reciprocal_;

        /** Whether to use the customized KD-Tree data structure. **/
        bool use_customized_tree_;

        // Approximate parameter in radius search.
        float approx_radius_para_;
        
        // Approximate parameter in NN search.
        float approx_nn_para_;

        /** Number of operations in the approximation search **/
        int approx_nn_ops_num_;

        /** Number of leaders in the approximation search **/
        int approx_leaders_num_;

        /** Number of leaders in the approximation search **/
        int approx_followers_num_;

        /** Maximum number of points in each leaf node (of the searching KDTree). **/
        int max_leaf_size_;

        /** Whether to save operations number, leader and follower info in approx search. **/
        bool save_approx_data_;

     };

    /** \brief @b CorrespondenceEstimation represents the base class for
      * determining correspondences between target and query point
      * sets/features.
      *
      * Code example:
      *
      * \code
      * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source, target;
      * // ... read or fill in source and target
      * pcl::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
      * est.setInputSource (source);
      * est.setInputTarget (target);
      *
      * pcl::Correspondences all_correspondences;
      * // Determine all reciprocal correspondences
      * est.determineReciprocalCorrespondences (all_correspondences);
      * \endcode
      *
      * \author Radu B. Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class CorrespondenceEstimation : public CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>
    {
      public:
        typedef boost::shared_ptr<CorrespondenceEstimation<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const CorrespondenceEstimation<PointSource, PointTarget, Scalar> > ConstPtr;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_reciprocal_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::treeH_source_ ;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::treeH_target_ ;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::points_target_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::points_target_ptr_; // target 
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::index_target_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::index_target_ptr_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::points_src_; // source
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::points_src_ptr_; // source
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::index_src_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::index_src_ptr_;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::buildHKdTree_target;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::buildHKdTree_source;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::setMaxLeafSize;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::max_leaf_size_;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::use_customized_tree_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::approx_nn_ops_num_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::approx_leaders_num_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::approx_followers_num_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::save_approx_data_;


        using PCLBase<PointSource>::deinitCompute;

        typedef pcl::search::KdTree<PointTarget> KdTree;
        typedef typename pcl::search::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceEstimation ()
        {
          corr_name_  = "CorrespondenceEstimation";
        }
      
        /** \brief Empty destructor */
        virtual ~CorrespondenceEstimation () 
        {
          delete treeH_target_;
          delete treeH_source_;
        }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ());

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ());

        
        /** \brief Clone and cast to CorrespondenceEstimationBase */
        virtual boost::shared_ptr< CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > 
        clone () const
        {
          Ptr copy (new CorrespondenceEstimation<PointSource, PointTarget, Scalar> (*this));
          return (copy);
        }

     };
  }
}

#include <pcl/registration/impl/correspondence_estimation.hpp>

#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_ */
