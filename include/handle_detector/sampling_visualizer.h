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

#ifndef SAMPLING_H
#define SAMPLING_H

#include "handle_detector/cylindrical_shell.h"
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

/** \brief SamplingVisualizer visualizes individual steps of importance sampling. This can be used to illustrate the
 * different sampling methods.
 * \author Andreas ten Pas
 */
class SamplingVisualizer
{
public:
  /**
   * Create a visual cylinder from a point on its axis, its axis, its radius, and its extent.
   * \param pt_on_axis a point on the cylinder's axis
   * \param axis_direction the vector that represents the cylinder's axis
   * \param radius the radius of the cylinder
   * \param extent the extent of the cylinder
   */
  pcl::ModelCoefficients
  createCylinder(Eigen::Vector3d pt_on_axis, Eigen::Vector3d axis_direction, double radius, double extent);

  /**
   * Add a set of cylindrical shells to a given pcl-viewer. The color for all shells is defined by the three
   * optional parameters.
   * \param shells the set of cylindrical shells to be added
   * \param viewer_void the pcl-viewer
   */
  void
  addCylinders(const std::vector<CylindricalShell> &shells, void* viewer_void, std::string handle_index = "", double r =
                   0.0,
               double g = 1.0, double b = 1.0);

  /**
   * Create a pcl-viewer that shows a point cloud, a set of cylindrical shells, and a set of samples.
   * \param cloud the cloud to be shown
   * \param shells the set of cylindrical shells to be shown
   * \param samples the set of samples to be shown
   * \param target_radius the target radius
   */
  void
  createViewer(PointCloud::ConstPtr cloud, std::vector<CylindricalShell> shells, Eigen::MatrixXd samples,
               double target_radius);

  /**
   * Create a pcl-viewer that shows a point cloud, a set of cylindrical shells, and a set of samples.
   * \param cloud the cloud to be shown
   * \param shells the set of cylindrical shells to be shown
   * \param samples the set of samples to be shown
   * \param target_radius the target radius
   */
  void
  createViewerRGB(PointCloudRGB::ConstPtr cloud, std::vector<CylindricalShell> shells, Eigen::MatrixXd samples,
                  double target_radius);

  /**
   * Returns the pcl-viewer.
   */
  inline boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer()
  {
    return this->viewer;
  }
  ;

private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif
