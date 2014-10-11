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

#ifndef PCL_FEATURES_CURVATURE_ESTIMATION_TAUBIN_H_
#define PCL_FEATURES_CURVATURE_ESTIMATION_TAUBIN_H_

#include <pcl/features/feature.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

// Lapack function to solve the generalized eigenvalue problem
extern "C" void dggev_(const char* JOBVL, const char* JOBVR, const int* N, const double* A, const int* LDA,
                       const double* B, const int* LDB, double* ALPHAR, double* ALPHAI, double* BETA, double* VL,
                       const int* LDVL, double* VR, const int* LDVR, double* WORK, const int* LWORK, int* INFO);

namespace pcl
{
/**
 * \ brief PCL point type to represent normal, curvature axis, curvature centroid, and median curvature
 */
struct PointCurvatureTaubin
{
  union
  {
    float normal[4];
    struct
    {
      float normal_x;
      float normal_y;
      float normal_z;
    };
  };
  union
  {
    float curvature_axis[4];
    struct
    {
      float curvature_axis_x;
      float curvature_axis_y;
      float curvature_axis_z;
    };
  };
  union
  {
    float curvature_centroid[4];
    struct
    {
      float curvature_centroid_x;
      float curvature_centroid_y;
      float curvature_centroid_z;
    };
  };
  union
  {
    float median_curvature;
  };EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure the allocators are aligned (SSE optimization)
}EIGEN_ALIGN16;

// size of matrices in Taubin Quadric Fitting
const int TAUBIN_MATRICES_SIZE = 10;

/** \brief CurvatureEstimationTaubin estimates the curvature for a set of point neighborhoods in
 * the cloud using Taubin Quadric Fitting. This class uses the OpenMP standard to permit
 * parallelized feature computation.
 * \author Andreas ten Pas
 * \ingroup features
 */
template<typename PointInT, typename PointOutT>
class CurvatureEstimationTaubin : public Feature<PointInT, PointOutT>
{
public:
  using Feature<PointInT, PointOutT>::feature_name_;
  using Feature<PointInT, PointOutT>::indices_;
  using Feature<PointInT, PointOutT>::input_;
  using Feature<PointInT, PointOutT>::surface_;
  using Feature<PointInT, PointOutT>::k_;
  using Feature<PointInT, PointOutT>::search_radius_;
  using Feature<PointInT, PointOutT>::search_parameter_;

  typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

  /** \brief Constructor. Set the number of threads to use.
   * \param num_threads the number of threads to use (0: automatic)
   */
  CurvatureEstimationTaubin(unsigned int num_threads = 0)
  {
    num_threads_ = num_threads;
    feature_name_ = "CurvatureEstimationTaubin";
  }

  /** \brief Fit a quadric to a given set of points, using their indices, and return the
   * parameters of the quadric in implicit form, its centroid, and its covariance matrix.
   * This method uses Taubin Quadric Fitting.
   * \param indices the point cloud indices of the points
   * \param quadric_parameters the resultant quadric parameters as: a, b, c, d, e, f, g, h, i,
   * j (ax^2 + by^2 + cz^2 + dxy + eyz + fxz + gx + hy + iz + j = 0)
   * \param quadric_centroid the resultant centroid of the quadric
   * \param quadric_covariance_matrix the resultant covariance matrix of the quadric
   */
  inline void fitQuadric(const std::vector<int> &indices, Eigen::VectorXd &quadric_parameters,
                         Eigen::Vector3d &quadric_centroid, Eigen::Matrix3d &quadric_covariance_matrix)
  {
    int n = indices.size();

    // calculate matrices M and N
    Eigen::Matrix<double, TAUBIN_MATRICES_SIZE, TAUBIN_MATRICES_SIZE> M;
    Eigen::Matrix<double, TAUBIN_MATRICES_SIZE, TAUBIN_MATRICES_SIZE> N;
    M.setZero(10, 10);
    N.setZero(10, 10);

    for (int i = 0; i < n; i++)
    {
      if (isnan(this->input_->points[indices[i]].x))
        continue;

      double x = this->input_->points[indices[i]].x;
      double y = this->input_->points[indices[i]].y;
      double z = this->input_->points[indices[i]].z;
      double x2 = x * x;
      double y2 = y * y;
      double z2 = z * z;
      double xy = x * y;
      double yz = y * z;
      double xz = x * z;

      // required calculations for M
      M(0, 0) += x2 * x2;
      M(0, 1) += x2 * y2;
      M(0, 2) += x2 * z2;
      M(0, 3) += x2 * xy;
      M(0, 4) += x2 * yz;
      M(0, 5) += x2 * xz;
      M(0, 6) += x2 * x;
      M(0, 7) += x2 * y;
      M(0, 8) += x2 * z;
      M(0, 9) += x2;
      M(1, 1) += y2 * y2;
      M(1, 2) += y2 * z2;
      M(1, 3) += y2 * xy;
      M(1, 4) += y2 * yz;
      M(1, 5) += y2 * xz;
      M(1, 6) += y2 * x;
      M(1, 7) += y2 * y;
      M(1, 8) += y2 * z;
      M(1, 9) += y2;
      M(2, 2) += z2 * z2;
      M(2, 3) += z2 * xy;
      M(2, 4) += z2 * yz;
      M(2, 5) += z2 * xz;
      M(2, 6) += z2 * x;
      M(2, 7) += z2 * y;
      M(2, 8) += z2 * z;
      M(2, 9) += z2;
      M(3, 8) += x * yz;
      M(3, 9) += xy;
      M(4, 9) += yz;
      M(5, 9) += xz;
      M(6, 9) += x;
      M(7, 9) += y;
      M(8, 9) += z;

      // repeating elements in M
      M(3, 3) = M(0, 1);
      M(5, 5) = M(0, 2);
      M(3, 5) = M(0, 4);
      M(3, 6) = M(0, 7);
      M(5, 6) = M(0, 8);
      M(6, 6) = M(0, 9);

      M(4, 4) = M(1, 2);
      M(3, 4) = M(1, 5);
      M(3, 7) = M(1, 6);
      M(4, 7) = M(1, 8);
      M(7, 7) = M(1, 9);

      M(4, 5) = M(2, 3);
      M(5, 8) = M(2, 6);
      M(4, 8) = M(2, 7);
      M(8, 8) = M(2, 9);

      M(4, 6) = M(3, 8);
      M(5, 7) = M(3, 8);
      M(6, 7) = M(3, 9);

      M(7, 8) = M(4, 9);

      M(6, 8) = M(5, 9);

      // required calculations for N
      N(0, 0) += 4 * x2;
      N(0, 3) += 2 * xy;
      N(0, 5) += 2 * xz;
      N(0, 6) += 2 * x;

      N(1, 1) += 4 * y2;
      N(1, 3) += 2 * xy;
      N(1, 4) += 2 * yz;
      N(1, 7) += 2 * y;

      N(2, 2) += 4 * z2;
      N(2, 4) += 2 * yz;
      N(2, 5) += 2 * xz;
      N(2, 8) += 2 * z;

      N(3, 3) += x2 + y2;
      N(3, 4) += xz;
      N(3, 5) += yz;
      N(3, 6) += y;
      N(3, 7) += x;

      N(4, 4) += y2 + z2;
      N(4, 5) += xy;
      N(4, 7) += z;
      N(4, 8) += y;

      N(5, 5) += x2 + z2;
      N(5, 6) += z;
      N(5, 8) += x;
    }

    M(9, 9) = n;
    // reflect upper triangular part in lower triangular part
    M.triangularView<Eigen::StrictlyLower>() = M.triangularView<Eigen::StrictlyUpper>().transpose();
    N(6, 6) = n;
    N(7, 7) = n;
    N(8, 8) = n;
    // reflect upper triangular part in lower triangular part
    N.triangularView<Eigen::StrictlyLower>() = N.triangularView<Eigen::StrictlyUpper>().transpose();

    // solve generalized Eigen problem to find quadric parameters
    Eigen::MatrixXd eigen_vectors;
    Eigen::MatrixXd lambda;
    this->solveGeneralizedEigenProblem(M, N, eigen_vectors, lambda);
    Eigen::VectorXd eigen_values = lambda.col(0).cwiseQuotient(lambda.col(2));
    int min_index;
    eigen_values.segment(0, 9).minCoeff(&min_index);
    quadric_parameters = eigen_vectors.col(min_index);
    quadric_parameters.segment(3, 3) *= 0.5;

    // compute centroid and covariance matrix of quadric
    this->unpackQuadric(quadric_parameters, quadric_centroid, quadric_covariance_matrix);
  }

  /** \brief Compares two vectors by looking at their second elements.
   * \param p1 the first vector to compare
   * \param p2 the second vector to compare
   */
  static inline bool isSecondElementSmaller(const std::vector<double>& p1, const std::vector<double>& p2)
  {
    return p1[1] < p2[1];
  }

  /** \brief Estimate the median curvature for a given quadric, using the indices of the point
   * neighborhood that the quadric is fitted to and its parameters, and return the estimated
   * curvature, the normal axis, the curvature axis, and the curvature centroid.
   * \param indices the point cloud indices of the points
   * \param quadric_parameters the quadric parameters as: a, b, c, d, e, f, g, h, i,
   * j (ax^2 + by^2 + cz^2 + dxy + eyz + fxz + gx + hy + iz + j = 0)
   * \param median_curvature the resultant, estimated median curvature of the quadric
   * \param normal the normal axis of the quadric (direction vector)
   * \param curvature_axis the curvature axis of the quadric (direction vector)
   * \param curvature_centroid the centroid of curvature
   */
  inline void estimateMedianCurvature(const std::vector<int> &indices, const Eigen::VectorXd &quadric_parameters,
                                      double &median_curvature, Eigen::Vector3d &normal,
                                      Eigen::Vector3d &curvature_axis, Eigen::Vector3d &curvature_centroid,
                                      bool is_deterministic = false)
  {
    // quadric parameters in implicit form
    double a = quadric_parameters(0);
    double b = quadric_parameters(1);
    double c = quadric_parameters(2);
    double d = 2 * quadric_parameters(3);
    double e = 2 * quadric_parameters(4);
    double f = 2 * quadric_parameters(5);
    double g = quadric_parameters(6);
    double h = quadric_parameters(7);
    double i = quadric_parameters(8);

    // create matrix to store <sample_num> samples that are close to the quadric
    Eigen::Matrix<double, 3, Eigen::Dynamic> samples_near_surf;
    int sample_num;

    // stochastic algorithm: look at a sample of 50 neighborhood points
    if (!is_deterministic)
    {
      sample_num = 50;
      Eigen::Matrix<double, 3, Eigen::Dynamic> samples_matrix(3, sample_num);

      for (int t = 0; t < sample_num; t++)
      {
        int r = rand() % indices.size();

        if (isnan(this->input_->points[indices[r]].x))
          continue;

        samples_matrix.col(t) << this->input_->points[indices[r]].x, this->input_->points[indices[r]].y, this->input_->points[indices[r]].z;
      }

      samples_near_surf = samples_matrix;
    }
    // deterministic algorithm: look at all points in the neighborhood
    else
    {
      Eigen::Matrix<double, 3, Eigen::Dynamic> samples_matrix(3, indices.size());
      sample_num = 0;

      for (std::size_t t = 0; t < indices.size(); t++)
      {
        if (isnan(this->input_->points[indices[t]].x))
          continue;

        samples_matrix(0, t) = this->input_->points[indices[t]].x;
        samples_matrix(1, t) = this->input_->points[indices[t]].y;
        samples_matrix(2, t) = this->input_->points[indices[t]].z;
        sample_num++;
      }

      samples_matrix.conservativeResize(3, sample_num);
      samples_near_surf = samples_matrix;
      sample_num = samples_near_surf.cols();
    }

    // calculate normals and gradient magnitude at each of these pts
    Eigen::MatrixXd fx =
        (2 * a * samples_near_surf.row(0) + d * samples_near_surf.row(1) + f * samples_near_surf.row(2)).array() + g;
    Eigen::MatrixXd fy =
        (2 * b * samples_near_surf.row(1) + d * samples_near_surf.row(0) + e * samples_near_surf.row(2)).array() + h;
    Eigen::MatrixXd fz =
        (2 * c * samples_near_surf.row(2) + e * samples_near_surf.row(1) + f * samples_near_surf.row(0)).array() + i;
    Eigen::MatrixXd normals(3, sample_num);
    normals << fx, fy, fz;
    Eigen::MatrixXd gradient_magnitude = ((normals.cwiseProduct(normals)).colwise().sum()).cwiseSqrt();
    normals = normals.cwiseQuotient(gradient_magnitude.replicate(3, 1));

    // calculate 2nd derivative of implicit quadric form
    Eigen::Matrix3d second_derivative_f;
    second_derivative_f << 2 * a, d, f, d, 2 * b, e, f, e, 2 * c;
    std::vector < std::vector<double> > list(sample_num, std::vector<double>(2));

    // iterate over the samples
    for (int t = 0; t < sample_num; t++)
    {
      // compute curvature by solving Eigen problem
      Eigen::Matrix3d curvature_matrix = -1
          * (Eigen::MatrixXd::Identity(3, 3) - normals.col(t) * normals.col(t).transpose()) * second_derivative_f
          / gradient_magnitude(t);
      Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(curvature_matrix);
      Eigen::Vector3d curvatures = eigen_solver.eigenvalues().real();
      int index;
      double max_coeff = curvatures.cwiseAbs().maxCoeff(&index);
      list[t][0] = t;
      list[t][1] = curvatures(index);
      normals.col(t) *= this->sign(list[t][1]);
      list[t][1] = fabs(list[t][1]);
    }

    // sort list of curvatures in increasing order
    std::sort(list.begin(), list.end(), isSecondElementSmaller);

    // compute median curvature
    int median_curvature_index = list[sample_num / 2 - 1][0];
    Eigen::Vector3d median_curvature_point = samples_near_surf.col(median_curvature_index);
    median_curvature = list[sample_num / 2 - 1][1];
    normal = normals.col(median_curvature_index);
    curvature_centroid = median_curvature_point + (normal / median_curvature);

    // find curvature axis by solving Eigen problem
    Eigen::Matrix3d curvature_matrix = (-1) * (Eigen::MatrixXd::Identity(3, 3) - normal * normal.transpose())
        * second_derivative_f / gradient_magnitude(median_curvature_index);
    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(curvature_matrix);
    Eigen::Matrix3d curvature_vectors = eigen_solver.eigenvectors().real();
    int max_index;
    eigen_solver.eigenvalues().real().cwiseAbs().maxCoeff(&max_index);
    curvature_axis = curvature_vectors.col(max_index).cross(normal);
  }

  /** \brief Set the number of samples (point neighborhoods).
   * \param num_samples the number of samples
   */
  inline void setNumSamples(int num_samples)
  {
    num_samples_ = num_samples;
  }

  /** \brief Set the number of threads used for parallelizing Taubin Quadric Fitting.
   * \param num_samples the number of samples
   */
  inline void setNumThreads(int num_threads)
  {
    num_threads_ = num_threads;
  }

  /** \brief Get the indices of each point neighborhood.
   */
  inline std::vector<std::vector<int> > const &getNeighborhoods() const
  {
    return neighborhoods_;
  }
  ;

  /** \brief Get the centroid indices of each point neighborhood.
   */
  inline std::vector<int> const &getNeighborhoodCentroids() const
  {
    return neighborhood_centroids_;
  }
  ;

  /** \brief Estimate the curvature for a set of point neighborhoods with given centroids.
   * \param samples the centroids of the neighborhoods
   * \param output the resultant point cloud that contains the curvature, normal axes,
   * curvature axes, and curvature centroids
   */
  void
  computeFeature(const Eigen::MatrixXd &samples, PointCloudOut &output);

protected:

  /** \brief Estimate the curvature for a set of point neighborhoods sampled from the cloud
   * given by <setInputCloud()>, using the spatial locator in setSearchMethod(). This method
   * creates its own set of randomly selected indices.
   * \note If <setIndices()> is used, the set of indices is not randomly selected.
   * \param output the resultant point cloud that contains the curvature, normal axes,
   * curvature axes, and curvature centroids
   */
  void
  computeFeature(PointCloudOut &output);

private:

  /** \brief Estimate the curvature for a set of points, using their indices and the index of
   * the neighborhood's centroid, and updates the output point cloud.
   * \param output the resultant point cloud that contains the curvature, normal axes,
   * curvature axes, and curvature centroids
   */
  void
  computeFeature(const std::vector<int> &nn_indices, int index, PointCloudOut &output);

  /** \brief Unpack the quadric, using its parameters, and return the centroid and the
   * covariance matrix of the quadric.
   * \param quadric_parameters the resultant quadric parameters as: a, b, c, d, e, f, g, h, i,
   * j (ax^2 + by^2 + cz^2 + dxy + eyz + fxz + gx + hy + iz + j = 0)
   * \param quadric_centroid the resultant centroid of the quadric
   * \param quadric_covariance_matrix the resultant covariance matrix of the quadric
   */
  inline void unpackQuadric(const Eigen::VectorXd &quadric_parameters, Eigen::Vector3d &quadric_centroid,
                            Eigen::Matrix3d &quadric_covariance_matrix)
  {
    double a = quadric_parameters(0);
    double b = quadric_parameters(1);
    double c = quadric_parameters(2);
    double d = quadric_parameters(3);
    double e = quadric_parameters(4);
    double f = quadric_parameters(5);
    double g = quadric_parameters(6);
    double h = quadric_parameters(7);
    double i = quadric_parameters(8);
    double j = quadric_parameters(9);

    Eigen::Matrix3d parameter_matrix;
    parameter_matrix << a, d, f, d, b, e, f, e, c;
    Eigen::Vector3d ghi;
    ghi << g, h, i;
    Eigen::Matrix3d inverse_parameter_matrix = parameter_matrix.inverse();
    quadric_centroid = -0.5 * inverse_parameter_matrix * ghi;
    double k = j - 0.25 * ghi.transpose() * inverse_parameter_matrix * ghi;
    quadric_covariance_matrix = -1 * parameter_matrix / k;
  }

  /** \brief Get the sign of a given value (according to the sign function).
   * \param x the given value
   */
  inline int sign(double x)
  {
    if (x > 0)
      return 1;
    if (x < 0)
      return -1;
    return 0;
  }

  /** \brief Solves the generalized Eigen problem A * v(j) = lambda(j) * B * v(j), where v
   * are the Eigen vectors, and lambda are the Eigen values. The eigenvalues are stored as:
   * (lambda(:, 1) + lambda(:, 2)*i)./lambda(:, 3). This method returns true if the Eigen
   * problem is solved successfully.
   * \param A the matrix A in the problem
   * \param B the matrix B in the problem
   * \param v the resultant Eigen vectors
   * \param lambda the resultant Eigen vectors (see above)
   */
  inline bool solveGeneralizedEigenProblem(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& v,
                                           Eigen::MatrixXd& lambda)
  {
    int N = A.cols(); // Number of columns of A and B. Number of rows of v.
    if (B.cols() != N || A.rows() != N || B.rows() != N)
      return false;

    v.resize(N, N);
    lambda.resize(N, 3);

    int LDA = A.outerStride();
    int LDB = B.outerStride();
    int LDV = v.outerStride();

    double WORKDUMMY;
    int LWORK = -1; // Request optimum work size.
    int INFO = 0;

    double* alphar = const_cast<double*>(lambda.col(0).data());
    double* alphai = const_cast<double*>(lambda.col(1).data());
    double* beta = const_cast<double*>(lambda.col(2).data());

    // Get the optimum work size.
    dggev_("N", "V", &N, A.data(), &LDA, B.data(), &LDB, alphar, alphai, beta, 0, &LDV, v.data(), &LDV, &WORKDUMMY,
           &LWORK, &INFO);

    LWORK = int(WORKDUMMY) + 32;
    Eigen::VectorXd WORK(LWORK);

    dggev_("N", "V", &N, A.data(), &LDA, B.data(), &LDB, alphar, alphai, beta, 0, &LDV, v.data(), &LDV, WORK.data(),
           &LWORK, &INFO);

    return INFO == 0;
  }
  ;

  unsigned int num_samples_; // number of samples (neighborhoods)
  unsigned int num_threads_; // number of threads for parallelization
  std::vector<std::vector<int> > neighborhoods_; // list of lists of point cloud indices for each neighborhood
  std::vector<int> neighborhood_centroids_; // list of point cloud indices corresponding to neighborhood centroids
  double time_taubin;
  double time_curvature;
};
}

#endif // PCL_FEATURES_CURVATURE_ESTIMATION_TAUBIN_H_
