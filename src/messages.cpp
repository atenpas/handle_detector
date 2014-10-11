#include <handle_detector/messages.h>

handle_detector::CylinderArrayMsg Messages::createCylinderArray(const std::vector<CylindricalShell> &list,
                                                                std::string frame)
{
  handle_detector::CylinderArrayMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame;
  msg.cylinders.resize(list.size());

  for (std::size_t i = 0; i < list.size(); i++)
  {
    msg.cylinders[i] = createCylinder(list[i], frame);
  }

  return msg;
}

handle_detector::CylinderMsg Messages::createCylinder(const CylindricalShell &shell, std::string frame)
{
  geometry_msgs::Pose pose;

  // cylinder position
  pose.position.x = shell.getCentroid()(0);
  pose.position.y = shell.getCentroid()(1);
  pose.position.z = shell.getCentroid()(2);

  // create cylinder orientation from axes
  geometry_msgs::PoseStamped cylinder_pose_msg;
  Eigen::Vector3d axis_eigen = shell.getCurvatureAxis();
  Eigen::Vector3d normal_eigen = shell.getNormal();
  Eigen::Vector3d perp_eigen = normal_eigen.cross(axis_eigen);
  tf::Matrix3x3 rotation_matrix(perp_eigen(0), normal_eigen(0), axis_eigen(0), perp_eigen(1), normal_eigen(1),
                                axis_eigen(1), perp_eigen(2), normal_eigen(2), axis_eigen(2));
  tf::Quaternion quaternion;
  rotation_matrix.getRotation(quaternion);
  tf::Stamped<tf::Transform> cylinder_tf_pose(tf::Transform(quaternion), ros::Time::now(), frame);
  tf::poseStampedTFToMsg(cylinder_tf_pose, cylinder_pose_msg);
  pose.orientation = cylinder_pose_msg.pose.orientation;

  // create message
  handle_detector::CylinderMsg cylinder_msg;
  cylinder_msg.pose = pose;
  cylinder_msg.radius = shell.getRadius();
  cylinder_msg.extent = shell.getExtent();
  geometry_msgs::Vector3 axis_msg;
  tf::vectorEigenToMsg(axis_eigen, axis_msg);
  geometry_msgs::Vector3 normal_msg;
  tf::vectorEigenToMsg(normal_eigen, normal_msg);
  cylinder_msg.axis = axis_msg;
  cylinder_msg.normal = normal_msg;
  return cylinder_msg;
}

handle_detector::HandleListMsg Messages::createHandleList(const std::vector<std::vector<CylindricalShell> > &handles,
                                                          std::string frame)
{
  handle_detector::HandleListMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame;
  msg.handles.resize(handles.size());

  for (std::size_t i = 0; i < handles.size(); i++)
    msg.handles[i] = createCylinderArray(handles[i], frame);

  return msg;
}
