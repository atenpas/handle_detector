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

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include "handle_detector/affordances.h"
#include "handle_detector/cylindrical_shell.h"
#include "handle_detector/sampling_visualizer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

/** \brief Sampling localizes grasp affordances using importance sampling.
 * \author Andreas ten Pas
 */
class Sampling
{
public:
  void setAffordances(Affordances &affordances)
  {
    this->affordances = affordances;
  }
  ;

  /**
   * Initializes the importance sampling parameters based on the parameters given
   * by a ROS node.
   * \param node the ROS node
   */
  void
  initParams(const ros::NodeHandle& node);

  /**
   * Illustrates the different sampling methods.
   * \param cloud the point cloud
   * \param cloudrgb the colored point cloud
   * \param target_radius the target radius
   */
  void
  illustrate(const PointCloud::Ptr &cloud, const PointCloudRGB::Ptr &cloudrgb, double target_radius);

  /**
   * Search affordances using importance sampling.
   * \param cloud the point cloud
   * \param cloudrgb the colored point cloud
   * \param target_radius the target radius
   */
  std::vector<CylindricalShell>
  searchAffordances(const PointCloud::Ptr &cloud, const PointCloudRGB::Ptr &cloudrgb, double target_radius);

private:
  Affordances affordances;
  int num_iterations;
  int num_samples;
  int num_init_samples;
  double prob_rand_samples;
  bool is_visualized;
  int method;

  // standard parameters
  static const int NUM_ITERATIONS;
  static const int NUM_SAMPLES;
  static const int NUM_INIT_SAMPLES;
  static const double PROB_RAND_SAMPLES;
  static const bool VISUALIZE_STEPS;
  static const int METHOD;
};

#endif /* SAMPLING_H_ */
