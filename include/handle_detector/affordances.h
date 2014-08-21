/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Andreas ten Pas
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
 */

#ifndef AFFORDANCES_H
#define AFFORDANCES_H

#include <fstream>
#include <iostream>
#include <omp.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <string>
#include <tf/transform_datatypes.h>
#include "curvature_estimation_taubin.h"
#include "curvature_estimation_taubin.hpp"
#include "cylindrical_shell.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

// workspace limits of the robot
struct WorkspaceLimits
{
	double min_x;
	double max_x;
	double min_y;
	double max_y;
	double min_z;
	double max_z;
};

/** \brief Affordances localizes grasp affordances and handles in a point cloud. It also provides 
  * helper methods to filter out points from the point cloud that are outside of the robot's 
  * workspace.
  * \author Andreas ten Pas
  */
class Affordances
{
	public:

    /** \brief Read the parameters from a ROS launch file.
      * \param node the ROS node with which the parameters are associated
      */ 
		void 
    initParams(ros::NodeHandle node);
    
    /** \brief Filter out all points from a given cloud that are outside of a sphere with a radius 
     * max_range and center at the origin of the point cloud, and return the filtered cloud.
     * \param cloud_in the point cloud to be filtered
     */ 
		PointCloud::Ptr 
    maxRangeFilter(const PointCloud::Ptr &cloud_in);
    
    /** \brief Filter out all points from a given cloud that are outside of a cube defined by the 
     * workspace limits of the robot, and return the filtered cloud.
     * \param cloud_in the point cloud to be filtered
     */
		PointCloud::Ptr 
    workspaceFilter(const PointCloud::Ptr &cloud_in, tf::StampedTransform *transform = NULL);
    
		/** \brief Filter out all points from a given cloud that are outside of a cube defined by the
     * workspace limits of the robot, and return the filtered cloud.
     * \param cloud_in the point cloud to be filtered
     */
    PointCloudRGB::Ptr 
    workspaceFilter(const PointCloudRGB::Ptr &cloud_in, tf::StampedTransform *transform = NULL);
        
    /** \brief Search grasp affordances (cylindrical shells) in a given point cloud.
     * \param cloud the point cloud in which affordances are searched for
     */
    std::vector<CylindricalShell> 
    searchAffordances(const PointCloud::Ptr &cloud, tf::StampedTransform *transform = NULL);
    
    /** \brief Search grasp affordances (cylindrical shells) using a set of indices in a given point cloud.
     * \param cloud the point cloud in which affordances are searched for
     * \paran indices the point cloud indices at which affordances are searched for
     */
    std::vector<CylindricalShell> 
    searchAffordances(const PointCloud::Ptr &cloud, const std::vector<int> &indices);
    
    /** \brief Search grasp affordances (cylindrical shells) using a set of samples in a given point cloud.
     * This function uses Taubin Quadric Fitting.
     * \param cloud the point cloud in which affordances are searched for
     * \param samples a 3xn matrix of points sampled from the point cloud
     */
    std::vector<CylindricalShell> 
    searchAffordancesTaubin(const PointCloud::Ptr &cloud, const Eigen::MatrixXd &samples, 
      bool is_logging = true);
        
    /** \brief Search handles in a set of cylindrical shells. If occlusion filtering is turned on 
     * (using the corresponding parameter in the ROS launch file), the handles found are filtered 
     * on possible occlusions.
     * \param cloud the point cloud in which the handles lie (only required for occlusion filtering)
     * \param shells the set of cylindrical shells to be searched for handles
     */
    std::vector< std::vector<CylindricalShell> > 
    searchHandles(const PointCloud::Ptr &cloud, std::vector<CylindricalShell> shells);
    
    std::vector<int> 
    createRandomIndices(const PointCloud::Ptr &cloud, int size);
    
    /** \brief Check whether a given point, using its x, y, and z coordinates, is within the 
     * workspace of the robot.
     * \param x the x coordinate of the point
     * \param y the y coordinate of the point
     * \param z the z coordinate of the point
     */ 
		bool 
    isPointInWorkspace(double x, double y, double z, tf::StampedTransform *transform = NULL);
    
    /** \brief Return the *.pcd file given by the corresponding parameter in the ROS launch file.
      */ 
    std::string getPCDFile() { return this->file; }
    
    /** \brief Return the number of samples, i.e., the number of neighborhoods to be searched for.
    */
    int getNumSamples() { return this->num_samples; }
    
    /** \brief Return the target radius.
    */
    double getTargetRadius() { return this->target_radius; }
      
  
	private:    
  
    /** \brief Estimate surface normals for each point in the point cloud.
     * \param cloud the point cloud for which surface normals are estimated
     * \param cloud_normals the resultant point cloud that contains the surface normals
     */
    void 
    estimateNormals(const PointCloud::Ptr &cloud, 
                    const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals);
    
    /** \brief Estimate the cylinder's curvature axis and normal using PCA.
     * \param cloud the point cloud in which the cylinder lies
     * \param nn_center_idx the index of the centroid of the neighborhood associated with the 
     * cylinder
     * \param nn_indices the point cloud indices of the neighborhood
     * \param axis the resultant curvature axis of the cylinder
     * \param normal the resultant normal of the cylinder
     */
		void 
    estimateCurvatureAxisPCA(const PointCloud::Ptr &cloud, int nn_center_idx, 
                            std::vector<int> nn_indices, Eigen::Vector3d &axis, 
                            Eigen::Vector3d &normal);

    /** \brief Estimate the cylinder's curvature axis and normal using surface normals.
     * \param cloud the point cloud in which the cylinder lies
     * \param nn_indices the point cloud indices of the neighborhood
     * \param axis the resultant curvature axis of the cylinder
     * \param normal the resultant normal of the cylinder
     */
    void 
    estimateCurvatureAxisNormals(const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, 
                                const std::vector<int> &nn_indices, 
                                Eigen::Vector3d &axis, Eigen::Vector3d &normal);
  
    /** \brief Search grasp affordances (cylindrical shells) in a given point cloud using surface 
     * normals.
     * \param cloud the point cloud in which affordances are searched for
     */
    std::vector<CylindricalShell> 
    searchAffordancesNormalsOrPCA(const PointCloud::Ptr &cloud, 
      tf::StampedTransform *transform = NULL);
				
    /** \brief Search grasp affordances (cylindrical shells) in a given point cloud using Taubin 
     * Quadric Fitting.
     * \param cloud the point cloud in which affordances are searched for
     */
    std::vector<CylindricalShell> 
    searchAffordancesTaubin(const PointCloud::Ptr &cloud, tf::StampedTransform *transform = NULL);
    
    /** \brief Find the best (largest number of inliers) set of colinear cylindrical shells given a 
     * list of shells.
     * \param list the list of cylindrical shells
     * \param inliersMaxSet the largest set of inliers found (the best colinear set)
     * \param outliersMaxSet the largest set of outliers found (the remaining shells that are not 
     * in the best colinear set)
     */
    void findBestColinearSet(const std::vector<CylindricalShell> &list, 
                            std::vector<int> &inliersMaxSet, 
                            std::vector<int> &outliersMaxSet);
                            
     /** \brief Find the number of points in the point cloud that lie in front of a point 
     * neighborhood with a given centroid index. A sphere with a given radius is searched for these 
     * points.
     * \param cloud the point cloud
     * \param center_index the index of the neighborhood centroid in the point cloud
     * \param radius the radius of the sphere
     */
		int numInFront(const PointCloud::Ptr &cloud, int center_index, double radius);

    // parameters (read-in from ROS launch file)
		double target_radius;
		double radius_error;
		double handle_gap;
		int num_samples;
		double max_range;
		bool use_clearance_filter;
		bool use_occlusion_filter;
		int curvature_estimator;
		int alignment_runs;
		int alignment_min_inliers;
		double alignment_dist_radius;
		double alignment_orient_radius;
		double alignment_radius_radius;
		WorkspaceLimits workspace_limits;
		int num_threads;
    std::string file;
    		
		// standard parameters
		static const int CURVATURE_ESTIMATOR; // curvature axis estimation method
		static const int NUM_SAMPLES; // number of neighborhoods
		static const int NUM_NEAREST_NEIGHBORS; // number of nearest neighbors to be found
		static const double NEIGHBOR_RADIUS;
		static const int MAX_NUM_IN_FRONT; // max. threshold of allowed points in front of a neighborhood center point (occlusion filtering)
		static const double TARGET_RADIUS; // approx. radius of the target handle
		static const double RADIUS_ERROR; // allowed deviation from target radius
		static const double HANDLE_GAP; // min. gap around affordances
		static const double MAX_RANGE; // max. range of robot arms
		static const bool USE_CLEARANCE_FILTER; // whether the clearance filter is used
		static const bool USE_OCCLUSION_FILTER; // whether the occlusion filter is used
		static const int ALIGNMENT_RUNS; // number of RANSAC runs
		static const int ALIGNMENT_MIN_INLIERS; // min. number of inliers for colinear cylinder set
		static const double ALIGNMENT_DIST_RADIUS; // distance threshold
		static const double ALIGNMENT_ORIENT_RADIUS; // orientation threshold
		static const double ALIGNMENT_RADIUS_RADIUS; // radius threshold
		static const double WORKSPACE_MIN;
		static const double WORKSPACE_MAX;
};

#endif
