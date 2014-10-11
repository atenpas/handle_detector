#include <handle_detector/cylindrical_shell.h>

void CylindricalShell::fitCylinder(const PointCloud::Ptr &cloud, const std::vector<int> &indices,
                                   const Eigen::Vector3d &normal, const Eigen::Vector3d &curvature_axis)
{
  // rotate points into axis aligned coordinate frame
  int n = indices.size();
  Eigen::Matrix3d R;
  R.col(0) = normal;
  R.col(1) = curvature_axis;
  R.col(2) = normal.cross(curvature_axis);
  Eigen::Matrix<double, 3, Eigen::Dynamic> pts(3, n);
  for (int i = 0; i < n; i++)
    pts.col(i) << cloud->points[indices[i]].x, cloud->points[indices[i]].y, cloud->points[indices[i]].z;
  Eigen::MatrixXd Rpts = R.transpose() * pts;

  // fit circle
  Eigen::RowVectorXd x = Rpts.row(0);
  Eigen::RowVectorXd y = Rpts.row(1);
  Eigen::RowVectorXd z = Rpts.row(2);
  Eigen::RowVectorXd x2 = x.cwiseProduct(x);
  Eigen::RowVectorXd z2 = z.cwiseProduct(z);
  Eigen::RowVectorXd one = Eigen::MatrixXd::Ones(1, n);

  Eigen::MatrixXd l(3, n);
  l << x, z, one;
  Eigen::RowVectorXd gamma = x2 + z2;

  Eigen::Matrix3d Q = l * l.transpose();
  Eigen::Vector3d c = ((gamma.replicate(3, 1)).cwiseProduct(l)).rowwise().sum();

  Eigen::Vector3d paramOut = -1 * Q.inverse() * c;

  // compute circle parameters
  Eigen::Vector2d circleCenter = -0.5 * paramOut.segment(0, 2);
  double circleRadius = sqrt(0.25 * (paramOut(0) * paramOut(0) + paramOut(1) * paramOut(1)) - paramOut(2));
  double axisCoord = y.sum() / n;

  // get cylinder parameters from circle parameters
  Eigen::Vector3d centroid_cyl_no_rot;
  centroid_cyl_no_rot << circleCenter(0), axisCoord, circleCenter(1);
  this->centroid = R * centroid_cyl_no_rot;
  this->radius = circleRadius;
  this->extent = y.maxCoeff() - y.minCoeff();
  this->curvature_axis = curvature_axis;
  this->normal = normal;
}

bool CylindricalShell::hasClearance(const PointCloud::Ptr &cloud, const std::vector<int>& nn_indices,
                                    double maxHandAperture, double handleGap)
{
  int min_points_inner = 40; // min number of points required to be within the inner cylinder
  int gap_threshold = 5; // threshold below which the gap is considered to be large enough

  std::vector<int> cropped_indices;
  for (std::size_t i = 0; i < nn_indices.size(); i++)
  {
    Eigen::Vector3d cropped = cloud->points[nn_indices[i]].getVector3fMap().cast<double>();
    double axialDist = this->curvature_axis.dot(cropped - centroid);
    if (fabs(axialDist) < this->extent / 2)
      cropped_indices.push_back(i);
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> croppedPts(3, cropped_indices.size());
  for (std::size_t i = 0; i < cropped_indices.size(); i++)
    croppedPts.col(i) = cloud->points[nn_indices[cropped_indices[i]]].getVector3fMap().cast<double>();

  Eigen::MatrixXd normalDiff = (Eigen::MatrixXd::Identity(3, 3) - curvature_axis * curvature_axis.transpose())
      * (croppedPts - centroid.replicate(1, croppedPts.cols()));
  Eigen::VectorXd normalDist = normalDiff.cwiseProduct(normalDiff).colwise().sum().cwiseSqrt();

  /* increase cylinder radius until number of points in gap is smaller than <gap_threshold> and
   * number of points within the inner cylinder is larger than <min_points_inner> */
  for (double r = this->radius; r <= maxHandAperture; r += 0.001)
  {
    int numInGap = ((normalDist.array() > r) * (normalDist.array() < r + handleGap) == true).count();
    int numInside = (normalDist.array() <= r).count();
    //~ printf("numInGap: %i, numInside: %i, \n", numInGap, numInside);

    if (numInGap < gap_threshold && numInside > min_points_inner)
    {
      this->radius = r;
      return true;
    }
  }

  return false;
}
