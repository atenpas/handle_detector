#include <handle_detector/affordances.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const int TAUBIN = 0;
const int PCA = 1;
const int NORMALS = 2;

const std::string CURVATURE_ESTIMATORS[] = {"Taubin", "PCA", "Normals"};

const int Affordances::CURVATURE_ESTIMATOR = 0;
const int Affordances::NUM_SAMPLES = 5000;
const int Affordances::NUM_NEAREST_NEIGHBORS = 500;
const double Affordances::NEIGHBOR_RADIUS = 0.025;
const int Affordances::MAX_NUM_IN_FRONT = 20;
const double Affordances::TARGET_RADIUS = 0.08;
const double Affordances::RADIUS_ERROR = 0.013;
const double Affordances::HANDLE_GAP = 0.08;
const double Affordances::MAX_RANGE = 1.0;
const bool Affordances::USE_CLEARANCE_FILTER = true;
const bool Affordances::USE_OCCLUSION_FILTER = true;
const int Affordances::ALIGNMENT_RUNS = 3;
const int Affordances::ALIGNMENT_MIN_INLIERS = 10;
const double Affordances::ALIGNMENT_DIST_RADIUS = 0.02;
const double Affordances::ALIGNMENT_ORIENT_RADIUS = 0.1;
const double Affordances::ALIGNMENT_RADIUS_RADIUS = 0.003;
const double Affordances::WORKSPACE_MIN = -1.0;
const double Affordances::WORKSPACE_MAX = 1.0;

void Affordances::initParams(ros::NodeHandle node)
{
  // read parameters from ROS launch file
  std::string file_default = "";
  node.param("file", this->file, file_default);
  node.param("target_radius", this->target_radius, this->TARGET_RADIUS);
  node.param("target_radius_error", this->radius_error, this->RADIUS_ERROR);
  node.param("affordance_gap", this->handle_gap, this->HANDLE_GAP);
  node.param("sample_size", this->num_samples, this->NUM_SAMPLES);
  node.param("max_range", this->max_range, this->MAX_RANGE);
  node.param("use_clearance_filter", this->use_clearance_filter, this->USE_CLEARANCE_FILTER);
  node.param("use_occlusion_filter", this->use_occlusion_filter, this->USE_OCCLUSION_FILTER);
  node.param("curvature_estimator", this->curvature_estimator, this->CURVATURE_ESTIMATOR);
  node.param("alignment_runs", this->alignment_runs, this->ALIGNMENT_RUNS);
  node.param("alignment_min_inliers", this->alignment_min_inliers, this->ALIGNMENT_MIN_INLIERS);
  node.param("alignment_dist_radius", this->alignment_dist_radius, this->ALIGNMENT_DIST_RADIUS);
  node.param("alignment_orient_radius", this->alignment_orient_radius, this->ALIGNMENT_ORIENT_RADIUS);
  node.param("alignment_radius_radius", this->alignment_radius_radius, this->ALIGNMENT_RADIUS_RADIUS);
  node.param("workspace_min_x", this->workspace_limits.min_x, this->WORKSPACE_MIN);
  node.param("workspace_max_x", this->workspace_limits.max_x, this->WORKSPACE_MAX);
  node.param("workspace_min_y", this->workspace_limits.min_y, this->WORKSPACE_MIN);
  node.param("workspace_max_y", this->workspace_limits.max_y, this->WORKSPACE_MAX);
  node.param("workspace_min_z", this->workspace_limits.min_z, this->WORKSPACE_MIN);
  node.param("workspace_max_z", this->workspace_limits.max_z, this->WORKSPACE_MAX);
  node.param("num_threads", this->num_threads, 1);

  // print parameters
  printf("PARAMETERS\n");
  printf(" file: %s\n", this->file.c_str());
  printf(" target radius: %.3f\n", this->target_radius);
  printf(" target radius error: %.3f\n", this->radius_error);
  printf(" min. affordance gap: %.3f\n", this->handle_gap);
  printf(" number of samples: %i\n", this->num_samples);
  printf(" max. range: %.3f\n", this->max_range);
  printf(" use clearance filter: %s\n", this->use_clearance_filter ? "true" : "false");
  printf(" use occlusion filter: %s\n", this->use_occlusion_filter ? "true" : "false");
  printf(" curvature estimator: %s\n", CURVATURE_ESTIMATORS[this->curvature_estimator].c_str());
  printf(" number of alignment runs: %i\n", this->alignment_runs);
  printf(" min. number of alignment inliers: %i\n", this->alignment_min_inliers);
  printf(" alignment distance threshold: %.3f\n", this->alignment_dist_radius);
  printf(" alignment orientation threshold: %.3f\n", this->alignment_orient_radius);
  printf(" alignment radius threshold: %.3f\n", this->alignment_radius_radius);
  printf(" workspace_min_x: %.3f\n", this->workspace_limits.min_x);
  printf(" workspace_max_x: %.3f\n", this->workspace_limits.max_x);
  printf(" workspace_min_y: %.3f\n", this->workspace_limits.min_y);
  printf(" workspace_max_y: %.3f\n", this->workspace_limits.max_y);
  printf(" workspace_min_z: %.3f\n", this->workspace_limits.min_z);
  printf(" workspace_max_z: %.3f\n", this->workspace_limits.max_z);
  printf(" num_threads: %i\n", this->num_threads);
}

PointCloud::Ptr Affordances::maxRangeFilter(const PointCloud::Ptr &cloud_in)
{
  PointCloud::Ptr cloud_out(new PointCloud);

  for (std::size_t i = 0; i < cloud_in->points.size(); i++)
  {
    if (cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y
        + cloud_in->points[i].z * cloud_in->points[i].z < this->max_range * this->max_range)
      cloud_out->points.push_back(cloud_in->points[i]);
  }

  return cloud_out;
}

bool Affordances::isPointInWorkspace(double x, double y, double z, tf::StampedTransform *transform)
{
  if (transform != NULL)
  {
    tf::Vector3 v(x, y, z);
    tf::Vector3 tf_v = (*transform) * v;
    x = tf_v.getX();
    y = tf_v.getY();
    z = tf_v.getZ();
  }

  WorkspaceLimits limits = this->workspace_limits;

  if (x >= limits.min_x && x <= limits.max_x && y >= limits.min_y && y <= limits.max_y && z >= limits.min_z
      && z <= limits.max_z)
  {
    return true;
  }

  return false;
}

PointCloud::Ptr Affordances::workspaceFilter(const PointCloud::Ptr &cloud_in, tf::StampedTransform *transform)
{
  PointCloud::Ptr cloud_out(new PointCloud);

  for (std::size_t i = 0; i < cloud_in->points.size(); i++)
  {
    if (this->isPointInWorkspace(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, transform))
      cloud_out->points.push_back(cloud_in->points[i]);
  }

  return cloud_out;
}

PointCloudRGB::Ptr Affordances::workspaceFilter(const PointCloudRGB::Ptr &cloud_in, tf::StampedTransform *transform)
{
  PointCloudRGB::Ptr cloud_out(new PointCloudRGB);

  for (std::size_t i = 0; i < cloud_in->points.size(); i++)
  {
    if (this->isPointInWorkspace(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, transform))
      cloud_out->points.push_back(cloud_in->points[i]);
  }

  return cloud_out;
}

int Affordances::numInFront(const PointCloud::Ptr &cloud, int center_index, double radius)
{
  Eigen::Vector3f center = cloud->points[center_index].getVector3fMap();
  double dist_center = center.norm();
  double theta = atan(radius / dist_center);
  Eigen::Vector3f center_unit = center / dist_center;
  int num_in_front = 0;

  for (std::size_t i = 0; i < cloud->points.size(); i++)
  {
    if (isnan(cloud->points[i].x))
      continue;

    Eigen::Vector3f point = cloud->points[i].getVector3fMap();
    float point_norm = point.norm();
    Eigen::Vector3f point_unit = point / point_norm;

    if (fabs(point_unit.dot(center_unit)) < cos(theta))
      continue;

    if (point_norm < dist_center - radius)
      num_in_front++;
  }

  return num_in_front;
}

void Affordances::estimateCurvatureAxisPCA(const PointCloud::Ptr &cloud, int nn_center_idx, std::vector<int> nn_indices,
                                           Eigen::Vector3d &axis, Eigen::Vector3d &normal)
{
  Eigen::Matrix3f covar_mat;
  Eigen::Vector4f nn_centroid;
  nn_centroid << cloud->points[nn_center_idx].x, cloud->points[nn_center_idx].y, cloud->points[nn_center_idx].z, 0;

  pcl::computeCovarianceMatrix(*cloud, nn_indices, nn_centroid, covar_mat);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covar_mat);
  Eigen::Vector3f eig_vals = eigen_solver.eigenvalues();
  int max_index;
  eig_vals.maxCoeff(&max_index);
  axis = eigen_solver.eigenvectors().col(max_index).cast<double>();
  Eigen::Vector3d perp_axis;
  perp_axis << -axis(1), axis(0), 0;
  normal = axis.cross(perp_axis);
  normal /= normal.norm();
}

void Affordances::estimateCurvatureAxisNormals(const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                                               const std::vector<int> &nn_indices, Eigen::Vector3d &axis,
                                               Eigen::Vector3d &normal)
{
  Eigen::Matrix3d mat;
  mat.setZero();

  for (std::size_t j = 0; j < nn_indices.size(); j++)
  {
    Eigen::Vector3d normal;
    normal << cloud_normals->points[nn_indices[j]].normal[0], cloud_normals->points[nn_indices[j]].normal[1], cloud_normals->points[nn_indices[j]].normal[2];
    Eigen::Matrix3d matNormal = normal * normal.transpose();
    mat += matNormal;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(mat);
  Eigen::Vector3d eig_vals = eigen_solver.eigenvalues();
  int min_index;
  eig_vals.minCoeff(&min_index);
  axis = eigen_solver.eigenvectors().col(min_index);
  Eigen::Vector3d perp_axis;
  perp_axis << -axis(1), axis(0), 0;
  normal = axis.cross(perp_axis);
  normal /= normal.norm();
}

void Affordances::estimateNormals(const PointCloud::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
{
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setRadiusSearch(0.03);
  normal_estimator.compute(*cloud_normals);
}

std::vector<CylindricalShell> Affordances::searchAffordances(const PointCloud::Ptr &cloud,
                                                             tf::StampedTransform *transform)
{
  std::vector<CylindricalShell> shells;
  shells.resize(0);

  if (this->curvature_estimator == TAUBIN)
    shells = this->searchAffordancesTaubin(cloud, transform);
  else if (this->curvature_estimator == NORMALS)
    shells = this->searchAffordancesNormalsOrPCA(cloud, transform);
  else if (this->curvature_estimator == PCA)
    shells = this->searchAffordancesNormalsOrPCA(cloud, transform);

  return shells;
}

std::vector<CylindricalShell> Affordances::searchAffordancesNormalsOrPCA(const PointCloud::Ptr &cloud,
                                                                         tf::StampedTransform *transform)
{
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // estimate surface normals
  if (this->curvature_estimator == NORMALS)
  {
    double begin_time_normals_estimation = omp_get_wtime();
    printf("Estimating surface normals ...\n");
    this->estimateNormals(cloud, cloud_normals);
    printf(" elapsed time: %.3f sec\n", omp_get_wtime() - begin_time_normals_estimation);
  }

  // search cloud for a set of point neighborhoods
  double begin_time_axis = omp_get_wtime();
  printf("Estimating cylinder surface normal and curvature axis ...\n");
  pcl::PointXYZ searchPoint;
  std::vector<int> nn_indices;
  std::vector < std::vector<int> > neighborhoods(this->num_samples);
  std::vector<int> neighborhood_centroids(this->num_samples);
  std::vector<Eigen::Vector3d> normals(this->num_samples);
  std::vector<Eigen::Vector3d> curvature_axes(this->num_samples);

  // set-up an Organized Neighbor search
  if (cloud->isOrganized())
  {
    std::vector<float> nn_dists;
    pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr organized_neighbor(
        new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
    organized_neighbor->setInputCloud(cloud);
    std::srand(std::time(0)); // use current time as seed for random generator

    for (int i = 0; i < this->num_samples; i++)
    {
      // sample random point from the point cloud
      int r = std::rand() % cloud->points.size();

      while (!pcl::isFinite((*cloud)[r]) || !this->isPointInWorkspace((*cloud)[r].x, (*cloud)[r].y, (*cloud)[r].z))
        r = std::rand() % cloud->points.size();

      // estimate cylinder curvature axis and normal
      if (organized_neighbor->radiusSearch((*cloud)[r], this->NEIGHBOR_RADIUS, nn_indices, nn_dists) > 0)
      {
        if (this->curvature_estimator == NORMALS)
          this->estimateCurvatureAxisNormals(cloud_normals, nn_indices, curvature_axes[i], normals[i]);
        else if (this->curvature_estimator == PCA)
          this->estimateCurvatureAxisPCA(cloud, r, nn_indices, curvature_axes[i], normals[i]);

        neighborhoods[i] = nn_indices;
        neighborhood_centroids[i] = r;
      }
    }
  }
  else
  {
    std::vector<float> nn_dists;
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    std::srand(std::time(0)); // use current time as seed for random generator

    for (int i = 0; i < this->num_samples; i++)
    {
      // sample random point from the point cloud
      int r = std::rand() % cloud->points.size();

      while (!pcl::isFinite((*cloud)[r]) || !this->isPointInWorkspace((*cloud)[r].x, (*cloud)[r].y, (*cloud)[r].z))
        r = std::rand() % cloud->points.size();

      // estimate cylinder curvature axis and normal
      if (tree.radiusSearch((*cloud)[r], this->NEIGHBOR_RADIUS, nn_indices, nn_dists) > 0)
      {
        if (this->curvature_estimator == NORMALS)
          this->estimateCurvatureAxisNormals(cloud_normals, nn_indices, curvature_axes[i], normals[i]);
        else if (this->curvature_estimator == PCA)
          this->estimateCurvatureAxisPCA(cloud, r, nn_indices, curvature_axes[i], normals[i]);

        neighborhoods[i] = nn_indices;
        neighborhood_centroids[i] = r;
      }
    }
  }

  printf(" elapsed time: %.3f sec\n", omp_get_wtime() - begin_time_axis);

  // define lower and upper bounds on radius of cylinder
  double min_radius_cylinder = this->target_radius - this->radius_error;
  double max_radius_cylinder = this->target_radius + this->radius_error;

  if (this->use_clearance_filter)
    printf("Filtering on curvature, fitting cylinders, and filtering on low clearance ...\n");
  else
    printf("Filtering on curvature and fitting cylinders ...\n");

  double begin_time = omp_get_wtime();
  int cylinders_left_clearance = 0;
  Eigen::Vector3d normal;
  Eigen::Vector3d curvature_axis;
  Eigen::Vector3d curvature_centroid;
  std::vector<CylindricalShell> shells;

  double maxHandAperture = this->target_radius + this->radius_error;
  double outer_sample_radius = 1.5 * (maxHandAperture + this->handle_gap); // outer sample radius

  // for organized point clouds
  pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr organized_neighbor(
      new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
  // for unorganized point clouds
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  if (cloud->isOrganized())
    organized_neighbor->setInputCloud(cloud);
  else
    tree.setInputCloud(cloud);

  for (int i = 0; i < this->num_samples; i++)
  {
    // fit a cylinder to the neighborhood
    CylindricalShell shell;
    shell.fitCylinder(cloud, neighborhoods[i], normals[i], curvature_axes[i]);

    // set height of shell to 2 * <target_radius>
    shell.setExtent(2.0 * this->target_radius);

    // set index of centroid of neighborhood associated with the cylindrical shell
    shell.setNeighborhoodCentroidIndex(neighborhood_centroids[i]);

    // check cylinder radius against target radius
    if (shell.getRadius() > min_radius_cylinder && shell.getRadius() < max_radius_cylinder)
    {
      // filter on low clearance
      if (this->use_clearance_filter)
      {
        pcl::PointXYZ searchPoint;
        std::vector<int> nn_indices;
        std::vector<float> nn_dists;
        Eigen::Vector3d centroid = shell.getCentroid();
        searchPoint.x = centroid(0);
        searchPoint.y = centroid(1);
        searchPoint.z = centroid(2);
        int num_in_radius = 0;
        if (cloud->isOrganized())
          num_in_radius = organized_neighbor->radiusSearch(searchPoint, outer_sample_radius, nn_indices, nn_dists);
        else
          num_in_radius = tree.radiusSearch(searchPoint, outer_sample_radius, nn_indices, nn_dists);

        if ((num_in_radius > 0) && (shell.hasClearance(cloud, nn_indices, maxHandAperture, this->handle_gap)))
          shells.push_back(shell);
      }
      else
        shells.push_back(shell);
    }
  }

  printf(" elapsed time: %.3f sec\n", omp_get_wtime() - begin_time);
  if (this->use_clearance_filter)
    printf(" cylinders left after clearance filtering: %i\n", (int)shells.size());

  return shells;
}

std::vector<CylindricalShell> Affordances::searchAffordancesTaubin(const PointCloud::Ptr &cloud,
                                                                   tf::StampedTransform *transform)
{
  printf("Estimating curvature ...\n");
  double beginTime = omp_get_wtime();

  // set-up estimator
  pcl::CurvatureEstimationTaubin<pcl::PointXYZ, pcl::PointCurvatureTaubin> estimator;

  // set input source
  estimator.setInputCloud(cloud);

  // set radius search
  estimator.setRadiusSearch(this->NEIGHBOR_RADIUS);

  // set the number of samples
  estimator.setNumSamples(this->num_samples);

  // provide a set of neighborhood centroids
  std::vector<int> indices(this->num_samples);
  std::srand(std::time(0)); // use current time as seed for random generator
  int k;
  for (int i = 0; i < this->num_samples; i++)
  {
    int r = std::rand() % cloud->points.size();
    k = 0;
    while (!pcl::isFinite((*cloud)[r])
        || !this->isPointInWorkspace(cloud->points[r].x, cloud->points[r].y, cloud->points[r].z, transform))
    {
      r = std::rand() % cloud->points.size();
      k++;
      if (k == cloud->points.size()) // check that the cloud has finite points
      {
        printf("No finite points in cloud!\n");
        std::vector<CylindricalShell> shells;
        shells.resize(0);
        return shells;
      }
    }
    indices[i] = r;
  }
  boost::shared_ptr<std::vector<int> > indices_ptr(new std::vector<int>(indices));
  estimator.setIndices(indices_ptr);

  // set number of threads
  estimator.setNumThreads(this->num_threads);

  // output dataset
  pcl::PointCloud<pcl::PointCurvatureTaubin>::Ptr cloud_curvature(new pcl::PointCloud<pcl::PointCurvatureTaubin>);

  // compute median curvature, normal axis, curvature axis, and curvature centroid
  estimator.compute(*cloud_curvature);

  printf(" elapsed time: %.3f sec\n", omp_get_wtime() - beginTime);

  // define lower and upper bounds on radius of osculating sphere and cylinder
  double min_radius_osculating_sphere = this->target_radius - 2.0 * this->radius_error;
  double max_radius_osculating_sphere = this->target_radius + 2.0 * this->radius_error;
  double min_radius_cylinder = this->target_radius - this->radius_error;
  double max_radius_cylinder = this->target_radius + this->radius_error;

  if (this->use_clearance_filter)
    printf("Filtering on curvature, fitting cylinders, and filtering on low clearance ...\n");
  else
    printf("Filtering on curvature and fitting cylinders ...\n");

  double begin_time = omp_get_wtime();
  int cylinders_left_radius = 0;
  int cylinders_left_clearance = 0;
  Eigen::Vector3d normal;
  Eigen::Vector3d curvature_axis;
  std::vector<CylindricalShell> shells;

  double maxHandAperture = this->target_radius + this->radius_error;
  double outer_sample_radius = 1.5 * (maxHandAperture + this->handle_gap); // outer sample radius

  // for organized point clouds
  pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr organized_neighbor(
      new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
  // for unorganized point clouds
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  if (cloud->isOrganized())
    organized_neighbor->setInputCloud(cloud);
  else
    tree.setInputCloud(cloud);

  for (int i = 0; i < cloud_curvature->size(); i++)
  {
    // calculate radius of osculating sphere
    double radius = 1.0 / fabs(cloud_curvature->points[i].median_curvature);

    // filter out planar regions and cylinders that are too large
    if (radius > min_radius_osculating_sphere && radius < max_radius_osculating_sphere)
    {
      // fit a cylinder to the neighborhood
      normal << cloud_curvature->points[i].normal_x, cloud_curvature->points[i].normal_y, cloud_curvature->points[i].normal_z;
      curvature_axis << cloud_curvature->points[i].curvature_axis_x, cloud_curvature->points[i].curvature_axis_y, cloud_curvature->points[i].curvature_axis_z;
      CylindricalShell shell;
      shell.fitCylinder(cloud, estimator.getNeighborhoods()[i], normal, curvature_axis);

      // set height of shell to 2 * <target_radius>
      shell.setExtent(2.0 * this->target_radius);

      // set index of centroid of neighborhood associated with the cylindrical shell
      shell.setNeighborhoodCentroidIndex(estimator.getNeighborhoodCentroids()[i]);

      // check cylinder radius against target radius
      if (shell.getRadius() > min_radius_cylinder && shell.getRadius() < max_radius_cylinder)
      {
        cylinders_left_radius++;

        // filter on low clearance
        if (this->use_clearance_filter)
        {
          pcl::PointXYZ searchPoint;
          std::vector<int> nn_indices;
          std::vector<float> nn_dists;
          Eigen::Vector3d centroid = shell.getCentroid();
          searchPoint.x = centroid(0);
          searchPoint.y = centroid(1);
          searchPoint.z = centroid(2);
          int num_in_radius = 0;
          if (cloud->isOrganized())
            num_in_radius = organized_neighbor->radiusSearch(searchPoint, outer_sample_radius, nn_indices, nn_dists);
          else
            num_in_radius = tree.radiusSearch(searchPoint, outer_sample_radius, nn_indices, nn_dists);

          if ((num_in_radius > 0) && (shell.hasClearance(cloud, nn_indices, maxHandAperture, this->handle_gap)))
            shells.push_back(shell);
        }
        else
          shells.push_back(shell);
      }
    }
  }

  printf(" elapsed time: %.3f sec\n", omp_get_wtime() - begin_time);
  printf(" cylinders left after radius filtering: %i\n", cylinders_left_radius);
  if (this->use_clearance_filter)
    printf(" cylinders left after clearance filtering: %i\n", (int)shells.size());

  return shells;
}

std::vector<std::vector<CylindricalShell> > Affordances::searchHandles(const PointCloud::Ptr &cloud,
                                                                       std::vector<CylindricalShell> shells)
{
  std::vector < std::vector<CylindricalShell> > handles;

  // find colinear sets of cylinders
  if (this->alignment_runs > 0)
  {
    std::cout << "alignment search for colinear sets of cylinders (handles) ... " << std::endl;
    double beginTime = omp_get_wtime();
    std::vector<int> inliersMaxSet, outliersMaxSet;

    // linear search
    for (int i = 0; i < this->alignment_runs && shells.size() > 0; i++)
    {
      this->findBestColinearSet(shells, inliersMaxSet, outliersMaxSet);
      printf(" number of inliers in run %i: %i", i, (int)inliersMaxSet.size());

      if (inliersMaxSet.size() >= this->alignment_min_inliers)
      {
        // create handle from inlier indices
        std::vector<CylindricalShell> handle;
        for (std::size_t j = 0; j < inliersMaxSet.size(); j++)
        {
          int idx = inliersMaxSet[j];
          handle.push_back(shells[idx]);
        }

        // check for occlusions
        if (this->use_occlusion_filter)
        {
          int MAX_NUM_OCCLUDED = (int)handle.size() * 0.5; // 5
          int num_occluded = 0;
          bool is_occluded = false;

          for (std::size_t j = 0; j < handle.size(); j++)
          {
            if (this->numInFront(cloud, handle[j].getNeighborhoodCentroidIndex(),
                                 1.5 * this->target_radius + this->radius_error) > this->MAX_NUM_IN_FRONT)
            {
              num_occluded++;
              if (num_occluded > MAX_NUM_OCCLUDED)
              {
                is_occluded = true;
                break;
              }
            }
          }

          printf("  number of occluded affordances: %i; occluded: %s\n", num_occluded, is_occluded ? "true" : "false");

          if (!is_occluded)
            handles.push_back(handle);
        }
        else
        {
          handles.push_back(handle);
        }

        // prune list of cylindrical shells
        std::vector<CylindricalShell> remainder(outliersMaxSet.size());
        for (std::size_t j = 0; j < outliersMaxSet.size(); j++)
          remainder[j] = shells[outliersMaxSet[j]];
        shells = remainder;
        printf(", remaining cylinders: %i\n", (int)shells.size());
      }
      // do not check for occlusions
      else
      {
        break;
      }
    }

    printf(" elapsed time: %.3f\n", omp_get_wtime() - beginTime);
  }

  return handles;
}

std::vector<CylindricalShell> Affordances::searchAffordances(const PointCloud::Ptr &cloud,
                                                             const std::vector<int> &indices)
{
  Eigen::MatrixXd samples(3, indices.size());
  for (std::size_t i = 0; i < indices.size(); i++)
    samples.col(i) = cloud->points[indices[i]].getVector3fMap().cast<double>();

  return this->searchAffordancesTaubin(cloud, samples);
}

std::vector<CylindricalShell> Affordances::searchAffordancesTaubin(const PointCloud::Ptr &cloud,
                                                                   const Eigen::MatrixXd &samples, bool is_logging)
{
  if (is_logging)
    printf("Estimating curvature ...\n");

  double beginTime = omp_get_wtime();

  // set-up estimator
  pcl::PointCloud<pcl::PointCurvatureTaubin>::Ptr cloud_curvature(new pcl::PointCloud<pcl::PointCurvatureTaubin>);
  pcl::CurvatureEstimationTaubin<pcl::PointXYZ, pcl::PointCurvatureTaubin> estimator;
  estimator.setInputCloud(cloud);
  estimator.setRadiusSearch(this->NEIGHBOR_RADIUS);
  //~ estimator.setRadiusSearch(1.5*target_radius + radius_error);
  estimator.setNumThreads(this->num_threads);

  // compute median curvature, normal axis, curvature axis, and curvature centroid
  estimator.computeFeature(samples, *cloud_curvature);

  if (is_logging)
    printf(" elapsed time: %.3f sec, cylinders left: %i\n", omp_get_wtime() - beginTime,
           (int)cloud_curvature->points.size());

  // define lower and upper bounds on radius of osculating sphere and cylinder
  double min_radius_osculating_sphere = this->target_radius - 2.0 * this->radius_error;
  double max_radius_osculating_sphere = this->target_radius + 2.0 * this->radius_error;
  double min_radius_cylinder = this->target_radius - this->radius_error;
  double max_radius_cylinder = this->target_radius + this->radius_error;

  if (is_logging && this->use_clearance_filter)
    printf("Filtering on curvature, fitting cylinders, and filtering on low clearance ...\n");
  else if (is_logging)
    printf("Filtering on curvature and fitting cylinders ...\n");

  double begin_time = omp_get_wtime();
  int cylinders_left_radius = 0;
  int cylinders_left_clearance = 0;
  Eigen::Vector3d normal;
  Eigen::Vector3d curvature_axis;
  std::vector<CylindricalShell> shells;
  double tcyltotal = 0.0;
  double tcleartotal = 0.0;

  double maxHandAperture = this->target_radius + this->radius_error;
  double outer_sample_radius = 1.5 * (maxHandAperture + this->handle_gap); // outer sample radius

  // for organized point clouds
  pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr organized_neighbor(
      new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
  // for unorganized point clouds
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  if (cloud->isOrganized())
    organized_neighbor->setInputCloud(cloud);
  else
    tree.setInputCloud(cloud);

  //~ #ifdef _OPENMP
  //~ #pragma omp parallel for shared (cylinderList) firstprivate(cloud_curvature) private(radius, centroid_cyl, extent_cyl, normal, curvature_axis, curvature_centroid) num_threads(this->num_threads)
  //~ #endif
  for (int i = 0; i < cloud_curvature->size(); i++)
  {
    if (isnan(cloud_curvature->points[i].normal[0]))
      continue;

    // calculate radius of osculating sphere
    double radius = 1.0 / fabs(cloud_curvature->points[i].median_curvature);
    //~ printf("%i: radius of osculating sphere: %.4f\n", i, radius);

    // filter out planar regions and cylinders that are too large
    if (radius > min_radius_osculating_sphere && radius < max_radius_osculating_sphere)
    {
      // fit a cylinder to the neighborhood
      double tcyl0 = omp_get_wtime();
      normal << cloud_curvature->points[i].normal_x, cloud_curvature->points[i].normal_y, cloud_curvature->points[i].normal_z;
      curvature_axis << cloud_curvature->points[i].curvature_axis_x, cloud_curvature->points[i].curvature_axis_y, cloud_curvature->points[i].curvature_axis_z;
      CylindricalShell shell;
      shell.fitCylinder(cloud, estimator.getNeighborhoods()[i], normal, curvature_axis);
      tcyltotal += omp_get_wtime() - tcyl0;

      //~ printf(" radius of fitted cylinder: %.4f\n", shell.getRadius());

      // set height of shell to 2 * <target_radius>
      shell.setExtent(2.0 * this->target_radius);

      // set index of centroid of neighborhood associated with the cylindrical shell
      shell.setNeighborhoodCentroidIndex(estimator.getNeighborhoodCentroids()[i]);

      // check cylinder radius against target radius
      if (shell.getRadius() > min_radius_cylinder && shell.getRadius() < max_radius_cylinder)
      {
        cylinders_left_radius++;

        // filter on low clearance
        if (this->use_clearance_filter)
        {
          double tclear0 = omp_get_wtime();
          pcl::PointXYZ searchPoint;
          std::vector<int> nn_indices;
          std::vector<float> nn_dists;
          Eigen::Vector3d centroid = shell.getCentroid();
          searchPoint.x = centroid(0);
          searchPoint.y = centroid(1);
          searchPoint.z = centroid(2);
          int num_in_radius = 0;
          if (cloud->isOrganized())
            num_in_radius = organized_neighbor->radiusSearch(searchPoint, outer_sample_radius, nn_indices, nn_dists);
          else
            num_in_radius = tree.radiusSearch(searchPoint, outer_sample_radius, nn_indices, nn_dists);

          if ((num_in_radius > 0) && (shell.hasClearance(cloud, nn_indices, maxHandAperture, this->handle_gap)))
            shells.push_back(shell);

          tcleartotal += omp_get_wtime() - tclear0;
        }
        else
          shells.push_back(shell);
      }
    }
  }

  if (is_logging)
  {
    printf(" elapsed time: %.3f sec\n", omp_get_wtime() - begin_time);
    printf(" cylinders left after radius filtering: %i\n", cylinders_left_radius);
    if (this->use_clearance_filter)
      printf(" cylinders left after clearance filtering: %i\n", (int)shells.size());
    printf("  cylinder/circle fitting: %.3f sec\n", tcyltotal);
    printf("  shell search: %.3f sec\n", tcleartotal);
  }

  return shells;
}

std::vector<int> Affordances::createRandomIndices(const PointCloud::Ptr &cloud, int size)
{
  std::vector<int> indices(size);

  for (int i = 0; i < size; i++)
  {
    int r = std::rand() % cloud->points.size();
    while (!pcl::isFinite((*cloud)[r])
        || !this->isPointInWorkspace(cloud->points[r].x, cloud->points[r].y, cloud->points[r].z))
      r = std::rand() % cloud->points.size();
    indices[i] = r;
  }

  return indices;
}

void Affordances::findBestColinearSet(const std::vector<CylindricalShell> &list, std::vector<int> &inliersMaxSet,
                                      std::vector<int> &outliersMaxSet)
{
  int maxInliers = 0;
  double orientRadius2 = this->alignment_orient_radius * this->alignment_orient_radius;
  double distRadius2 = this->alignment_dist_radius * this->alignment_dist_radius;

  for (std::size_t i = 0; i < list.size(); i++)
  {
    Eigen::Vector3d axis = list[i].getCurvatureAxis();
    Eigen::Vector3d centroid = list[i].getCentroid();
    double radius = list[i].getRadius();
    std::vector<int> inliers, outliers;

    for (std::size_t j = 0; j < list.size(); j++)
    {
      Eigen::Vector3d distToOrientVec = (Eigen::MatrixXd::Identity(3, 3) - axis * axis.transpose())
          * list[j].getCurvatureAxis();
      double distToOrient = distToOrientVec.cwiseProduct(distToOrientVec).sum();
      Eigen::Vector3d distToAxisVec = (Eigen::MatrixXd::Identity(3, 3) - axis * axis.transpose())
          * (list[j].getCentroid() - centroid);
      double distToAxis = distToAxisVec.cwiseProduct(distToAxisVec).sum();
      double distToRadius = fabs(list[j].getRadius() - radius);

      if (distToOrient < orientRadius2 && distToAxis < distRadius2 && distToRadius < this->alignment_radius_radius)
        inliers.push_back(j);
      else
        outliers.push_back(j);
    }

    if (inliers.size() > maxInliers)
    {
      maxInliers = inliers.size();
      inliersMaxSet = inliers;
      outliersMaxSet = outliers;
    }
  }
}
