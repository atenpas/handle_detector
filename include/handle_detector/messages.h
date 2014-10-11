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

#ifndef MESSAGES_H
#define MESSAGES_H

#include "handle_detector/CylinderArrayMsg.h"
#include "handle_detector/CylinderMsg.h"
#include "handle_detector/HandleListMsg.h"
#include "cylindrical_shell.h"
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

/** \brief Messages creates custom ROS messages to publish the results of the localization. The 
 * messages that can be created are: CylinderArray, Cylinder, and HandleList.
 * \author Andreas ten Pas
 */
class Messages
{
public:
  /** \brief Create a CylinderArray message from a list of cylindrical shells.
   * \param list the list of cylindrical shells
   * \param frame the frame in which the shells are located
   */
  handle_detector::CylinderArrayMsg
  createCylinderArray(const std::vector<CylindricalShell> &list, std::string frame);

  /** \brief Create a Cylinder message from a cylindrical shell.
   * \param shell the cylindrical shell
   * \param frame the frame in which the shell is located
   */
  handle_detector::CylinderMsg
  createCylinder(const CylindricalShell &shell, std::string frame);

  /** \brief Create a HandleList message from a list of cylindrical shells.
   * \param handles the list of handles
   * \param frame the frame in which the shells are located
   */
  handle_detector::HandleListMsg
  createHandleList(const std::vector<std::vector<CylindricalShell> > &handles, std::string frame);
};

#endif
