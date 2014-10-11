#ifndef PCL_FEATURES_IMPL_CURVATURE_ESTIMATION_TAUBIN_HPP_
#define PCL_FEATURES_IMPL_CURVATURE_ESTIMATION_TAUBIN_HPP_

#include "curvature_estimation_taubin.h"

template<typename PointInT, typename PointOutT> void pcl::CurvatureEstimationTaubin<PointInT, PointOutT>::computeFeature(
    const Eigen::MatrixXd &samples, PointCloudOut &output)
{
  const double MIN_NEIGHBORS = 10;

  this->time_taubin = 0.0;
  this->time_curvature = 0.0;

  // allocate space to hold the indices and distances of the nearest neighbors
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;

  // the output only contains finite values
  output.is_dense = true;

  // the output contains features for <num_samples_> point neighborhoods
  output.resize(samples.cols());

  // resize neighborhoods to store neighborhoods
  neighborhoods_.resize(samples.cols());
  neighborhood_centroids_.resize(samples.cols());

  // set-up an Organized Neighbor search
  if (input_->isOrganized())
  {
    pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr organized_neighbor(
        new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
    organized_neighbor->setInputCloud(input_);
    pcl::PointXYZ search_point;

    // parallelization using OpenMP
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(num_threads_)
#endif
    // iterate over samples matrix
    for (int i = 0; i < samples.cols(); i++)
    {
      search_point.x = samples(0, i);
      search_point.y = samples(1, i);
      search_point.z = samples(2, i);
      //~ printf("  %i: (%.2f, %.2f, %.2f) \n", i, search_point.x, search_point.y, search_point.z);

      // find points that lie inside the cylindrical shell
      if (organized_neighbor->radiusSearch(search_point, search_radius_, nn_indices, nn_dists) < MIN_NEIGHBORS)
      {
        output.points[i].normal[0] = output.points[i].normal[1] = output.points[i].normal[2] =
            std::numeric_limits<float>::quiet_NaN();
        output.points[i].curvature_axis[0] = output.points[i].curvature_axis[1] = output.points[i].curvature_axis[2] =
            output.points[i].normal[0];
        output.points[i].curvature_centroid[0] = output.points[i].curvature_centroid[1] =
            output.points[i].curvature_centroid[2] = output.points[i].normal[0];
        output.points[i].median_curvature = output.points[i].normal[0];

        output.is_dense = false;
        continue;
      }
      else
      {
        //~ printf("i: %i, nn_indices.size: %i\n", i, (int) nn_indices.size());
        // compute feature at index using point neighborhood
        computeFeature(nn_indices, i, output);

        // store neighborhood for later processing
        neighborhoods_[i] = nn_indices;
        neighborhood_centroids_[i] = i;
      }
    }
  }
  else
  {
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    tree->setInputCloud(input_);
    pcl::PointXYZ search_point;

    // parallelization using OpenMP
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(num_threads_)
#endif
    // iterate over samples matrix
    for (int i = 0; i < samples.cols(); i++)
    {
      search_point.x = samples(0, i);
      search_point.y = samples(1, i);
      search_point.z = samples(2, i);
      //~ printf("  %i: (%.2f, %.2f, %.2f) \n", i, search_point.x, search_point.y, search_point.z);

      // find points that lie inside the cylindrical shell
      if (tree->radiusSearch(search_point, search_radius_, nn_indices, nn_dists) < MIN_NEIGHBORS)
      {
        output.points[i].normal[0] = output.points[i].normal[1] = output.points[i].normal[2] =
            std::numeric_limits<float>::quiet_NaN();
        output.points[i].curvature_axis[0] = output.points[i].curvature_axis[1] = output.points[i].curvature_axis[2] =
            output.points[i].normal[0];
        output.points[i].curvature_centroid[0] = output.points[i].curvature_centroid[1] =
            output.points[i].curvature_centroid[2] = output.points[i].normal[0];
        output.points[i].median_curvature = output.points[i].normal[0];

        output.is_dense = false;
        continue;
      }
      else
      {
        //~ printf("i: %i, nn_indices.size: %i\n", i, (int) nn_indices.size());
        // compute feature at index using point neighborhood
        computeFeature(nn_indices, i, output);

        // store neighborhood for later processing
        neighborhoods_[i] = nn_indices;
        neighborhood_centroids_[i] = i;
      }
    }
  }

  int n = 0;
  for (int i = 0; i < samples.cols(); i++)
  {
    if (neighborhoods_[i].size() >= MIN_NEIGHBORS)
      n++;
  }
  printf(" Taubin fitting: %.3f sec\n", this->time_taubin);
  printf(" Curvature estimation: %.3f sec\n", this->time_curvature);
}

template<typename PointInT, typename PointOutT> void pcl::CurvatureEstimationTaubin<PointInT, PointOutT>::computeFeature(
    PointCloudOut &output)
{
  // allocate space to hold the indices and distances of the nearest neighbors
  std::vector<int> nn_indices(k_);
  std::vector<float> nn_dists(k_);

  // the output only contains finite values
  output.is_dense = true;

  // the output contains features for <num_samples_> point neighborhoods
  output.resize(num_samples_);

  // if the cloud is dense, do not check for NaNs / infs (saves some computation cycles)
  if (input_->is_dense)
  {
    // if no indices given, create a random set of indices (neighborhood centroids)
    if (indices_->size() != num_samples_)
    {
      std::srand(std::time(0)); // use current time as seed for random generator
      indices_->resize(num_samples_);

      for (int i = 0; i < num_samples_; i++)
      {
        (*indices_)[i] = std::rand() % input_->points.size();
      }
    }
  }
  else // otherwise, check for NaNs and infs
  {
    // if no indices given, create a random set of indices (neighborhood centroids)
    if (indices_->size() != num_samples_)
    {
      std::srand(std::time(0)); // use current time as seed for random generator
      indices_->resize(num_samples_);

      for (int i = 0; i < num_samples_; i++)
      {
        int r = std::rand() % input_->points.size();

        while (!isFinite((*input_)[r]))
          r = std::rand() % input_->points.size();

        (*indices_)[i] = r;
      }
    }
  }

  // resize neighborhoods to store neighborhoods
  neighborhoods_.resize(indices_->size());
  neighborhood_centroids_.resize(indices_->size());

  // parallelization using OpenMP
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(num_threads_)
#endif

  // iterate over indices vector
  for (std::size_t idx = 0; idx < indices_->size(); ++idx)
  {
    if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
    {
      output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = std::numeric_limits<
          float>::quiet_NaN();
      output.points[idx].curvature_axis[0] = output.points[idx].curvature_axis[1] =
          output.points[idx].curvature_axis[2] = output.points[idx].normal[0];
      output.points[idx].curvature_centroid[0] = output.points[idx].curvature_centroid[1] =
          output.points[idx].curvature_centroid[2] = output.points[idx].normal[0];
      output.points[idx].median_curvature = output.points[idx].normal[0];

      output.is_dense = false;
      continue;
    }

    // compute feature at index using point neighborhood
    computeFeature(nn_indices, idx, output);

    // store neighborhood for later processing
    neighborhoods_[idx] = nn_indices;
    neighborhood_centroids_[idx] = (*indices_)[idx];
  }
}

template<typename PointInT, typename PointOutT> void pcl::CurvatureEstimationTaubin<PointInT, PointOutT>::computeFeature(
    const std::vector<int> &nn_indices, int index, PointCloudOut &output)
{
  // perform Taubin fit
  double t0 = omp_get_wtime();
  Eigen::VectorXd quadric_parameters(10);
  Eigen::Vector3d quadric_centroid;
  Eigen::Matrix3d quadric_covariance_matrix;
  this->fitQuadric(nn_indices, quadric_parameters, quadric_centroid, quadric_covariance_matrix);
  this->time_taubin += omp_get_wtime() - t0;

  // estimate median curvature, normal axis, curvature axis, and curvature centroid
  t0 = omp_get_wtime();
  double median_curvature;
  Eigen::Vector3d normal;
  Eigen::Vector3d curvature_axis;
  Eigen::Vector3d curvature_centroid;
  this->estimateMedianCurvature(nn_indices, quadric_parameters, median_curvature, normal, curvature_axis,
                                curvature_centroid);
  this->time_curvature += omp_get_wtime() - t0;

  // put median curvature, normal axis, curvature axis, and curvature centroid into cloud
  output[index].normal[0] = normal[0];
  output[index].normal[1] = normal[1];
  output[index].normal[2] = normal[2];
  output[index].curvature_axis[0] = curvature_axis[0];
  output[index].curvature_axis[1] = curvature_axis[1];
  output[index].curvature_axis[2] = curvature_axis[2];
  output[index].curvature_centroid[0] = curvature_centroid[0];
  output[index].curvature_centroid[1] = curvature_centroid[1];
  output[index].curvature_centroid[2] = curvature_centroid[2];
  output[index].median_curvature = median_curvature;
}

#endif // PCL_FEATURES_IMPL_CURVATURE_ESTIMATION_TAUBIN_HPP_
