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

#ifndef CYLINDRICAL_SHELL_H
#define CYLINDRICAL_SHELL_H

#include "Eigen/Dense"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/search/organized.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/** \brief CylindricalShell represents a cylindrical shell that consists of two colinear cylinders. 
 * A shell consists of an inner and an outer cylinder. The portion of the object to be grasped
 * must fit inside the inner cylinder, and the radius of that cylinder must be no larger than the
 * maximum hand aperture. The gap between the inner and outer cylinder must be free of obstacles
 * and wide enough to be able to contain the robot fingers.
 * \author Andreas ten Pas
 */
class CylindricalShell
{
public:

  /** \brief Fit a cylinder to a set of points in the cloud, using their indices, and the normal
   * and the curvature axis given by the quadric fitting (see curvature_estimation_taubin.h).
   * The fitted cylinder is the inner cylinder of the cylindrical shell.
   * \param cloud the point cloud
   * \param indices the indices of the set of points in the cloud
   * \param normal the normal given by quadric fitting
   * \param curvature_axis the curvature axis given by quadric fitting
   */
  void
  fitCylinder(const PointCloud::Ptr &cloud, const std::vector<int> &indices, const Eigen::Vector3d &normal,
              const Eigen::Vector3d &curvature_axis);

  /** \brief Check whether the gap between the inner and outer cylinder of the shell is free
   * of obstacles and wide enough to be able to contain the robot fingers.
   * \param cloud the point cloud
   * \param maxHandAperture the maximum robot hand aperture
   * \param handleGap the required size of the gap around the handle
   */
  bool
  hasClearance(const PointCloud::Ptr &cloud, const std::vector<int>& nn_indices, double maxHandAperture,
               double handleGap);

  /** \brief Get the extent of the cylindrical shell.
   */
  inline double getExtent() const
  {
    return this->extent;
  }
  ;

  /** \brief Set the extent of the cylindrical shell.
   * \param extent the extent
   */
  inline void setExtent(double extent)
  {
    this->extent = extent;
  }
  ;

  /** \brief Get the radius of the cylindrical shell.
   */
  inline double getRadius() const
  {
    return this->radius;
  }
  ;

  /** \brief Get the index of the centroid of the neighborhood associated with the cylindrical
   * shell.
   */
  inline int getNeighborhoodCentroidIndex() const
  {
    return this->neighborhood_centroid_index;
  }
  ;

  /** \brief Set the index of the centroid of the neighborhood associated with the cylindrical
   * shell.
   * \param index the index of the centroid
   */
  inline void setNeighborhoodCentroidIndex(int index)
  {
    this->neighborhood_centroid_index = index;
  }
  ;

  /** \brief Get the centroid of the cylindrical shell.
   */
  inline Eigen::Vector3d getCentroid() const
  {
    return this->centroid;
  }
  ;

  /** \brief Get the curvature axis of the cylindrical shell.
   */
  inline Eigen::Vector3d getCurvatureAxis() const
  {
    return this->curvature_axis;
  }
  ;

  /** \brief Get the normal axis of the cylindrical shell.
   */
  inline Eigen::Vector3d getNormal() const
  {
    return this->normal;
  }
  ;

private:

  Eigen::Vector3d centroid;
  Eigen::Vector3d curvature_axis;
  double extent;
  double radius;
  Eigen::Vector3d normal;
  int neighborhood_centroid_index;
};

#endif
