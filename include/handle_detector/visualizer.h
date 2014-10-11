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

#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <boost/lexical_cast.hpp>
#include "cylindrical_shell.h"
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef visualization_msgs::MarkerArray MarkerArray;

/** \brief Visualizer creates ROS messages to visualize the results of the localization in RViz. 
 * The possible objects that can be visualized are: neighborhoods, cylindrical shells, and handles.
 * \author Andreas ten Pas
 */
class Visualizer
{
public:
  /** \brief Constructor. Set the lifetime of markers in RViz.
   * \param num_threads the lifetime in seconds
   */
  Visualizer(double marker_lifetime);

  /** \brief Create a MarkerArray message from a list of cylindrical shells.
   * \param list the list of cylindrical shells
   * \param frame the frame in which the shells are located
   */
  MarkerArray
  createCylinders(const std::vector<CylindricalShell> &list, const std::string &frame);

  /** \brief Create a MarkerArray message from a list of cylindrical shells.
   * \param list the list of cylindrical shells
   * \param frame the frame in which the shells are located
   */
  MarkerArray
  createHandleNumbers(const std::vector<std::vector<CylindricalShell> > &handles, const std::string &frame);

  /** \brief Create a list of MarkerArray messages and a MarkerArray from a list of handles. The
   * former represents each handle as a MarkerArray message, and the latter represents all
   * handles in a single MarkerArray message.
   * \param handles the list of handles
   * \param frame the frame in which the handles are located
   * \param marker_arrays the resultant list of MarkerArray messages
   * \param all_handle_markers the resultant single MarkerArray message that consists of all
   * handles
   */
  void
  createHandles(const std::vector<std::vector<CylindricalShell> > &handles, const std::string &frame,
                std::vector<MarkerArray> &marker_arrays, MarkerArray &all_handle_markers);

private:
  double marker_lifetime;
};

#endif
