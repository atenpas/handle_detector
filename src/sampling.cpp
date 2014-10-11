#include "handle_detector/sampling.h"

// sampling methods
const int SUM = 1;
const int MAX = 2;

const int Sampling::NUM_ITERATIONS = 10;
const int Sampling::NUM_SAMPLES = 100;
const int Sampling::NUM_INIT_SAMPLES = 100;
const double Sampling::PROB_RAND_SAMPLES = 0.2;
const bool Sampling::VISUALIZE_STEPS = false;
const int Sampling::METHOD = SUM;

void Sampling::illustrate(const PointCloud::Ptr &cloud, const PointCloudRGB::Ptr &cloudrgb, double target_radius)
{
  // parameters
  int num_samples = 200;
  int num_init_samples = 1000;
  double sigma = 2.0 * target_radius;

  // find initial affordances
  std::vector<int> indices = this->affordances.createRandomIndices(cloud, num_init_samples);
  std::vector<CylindricalShell> all_shells = this->affordances.searchAffordances(cloud, indices);

  double term = 1.0 / sqrt(pow(2.0 * M_PI, 3.0) * pow(sigma, 3.0));

  // create random generator for normal distribution
  boost::mt19937 *rng = new boost::mt19937();
  rng->seed(time(NULL));
  boost::normal_distribution<> distribution(0.0, 1.0);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator(*rng, distribution);

  // sum of Gaussians
  Eigen::MatrixXd samples_sum(3, num_samples);
  for (int j = 0; j < num_samples; j++)
  {
    int idx = rand() % all_shells.size();
    samples_sum(0, j) = all_shells[idx].getCentroid()(0) + generator() * sigma;
    samples_sum(1, j) = all_shells[idx].getCentroid()(1) + generator() * sigma;
    samples_sum(2, j) = all_shells[idx].getCentroid()(2) + generator() * sigma;
  }

  // max of Gaussians
  Eigen::MatrixXd samples_max(3, num_samples);
  int j = 0;
  int count[all_shells.size()];
  for (std::size_t k = 0; k < all_shells.size(); k++)
    count[k] = 0;
  while (j < num_samples) // draw samples using rejection sampling
  {
    // draw from sum of Gaussians
    int idx = rand() % all_shells.size();
    Eigen::Vector3d x;
    x(0) = all_shells[idx].getCentroid()(0) + generator() * sigma;
    x(1) = all_shells[idx].getCentroid()(1) + generator() * sigma;
    x(2) = all_shells[idx].getCentroid()(2) + generator() * sigma;

    double maxp = 0;
    for (std::size_t k = 0; k < all_shells.size(); k++)
    {
      double p = (x - all_shells[k].getCentroid()).transpose() * (x - all_shells[k].getCentroid());
      p = term * exp((-1.0 / (2.0 * sigma)) * p);
      if (p > maxp)
        maxp = p;
    }

    double p = (x - all_shells[idx].getCentroid()).transpose() * (x - all_shells[idx].getCentroid());
    p = term * exp((-1.0 / (2.0 * sigma)) * p);
    if (p >= maxp)
    {
      samples_max.col(j) = x;
      count[idx]++;
      j++;
    }
  }

  SamplingVisualizer sum_visualizer;
  sum_visualizer.createViewerRGB(cloudrgb, all_shells, samples_sum, target_radius);
  sum_visualizer.getViewer()->setSize(800, 600);
  sum_visualizer.getViewer()->setCameraPosition(0.255106, -0.903705, -0.538521, 0.255053, -0.902864, -0.537242,
                                                -0.0206334, -0.835517, 0.549076);
  sum_visualizer.getViewer()->addText("Samples drawn from sum of Gaussians", 10, 20, 16, 1.0, 1.0, 1.0);
  while (!sum_visualizer.getViewer()->wasStopped())
  {
    sum_visualizer.getViewer()->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  SamplingVisualizer max_visualizer;
  max_visualizer.createViewerRGB(cloudrgb, all_shells, samples_max, target_radius);
  max_visualizer.getViewer()->setSize(800, 600);
  max_visualizer.getViewer()->setCameraPosition(0.255106, -0.903705, -0.538521, 0.255053, -0.902864, -0.537242,
                                                -0.0206334, -0.835517, 0.549076);
  max_visualizer.getViewer()->addText("Samples drawn from sum of Gaussians", 10, 20, 16, 1.0, 1.0, 1.0);
  while (!max_visualizer.getViewer()->wasStopped())
  {
    max_visualizer.getViewer()->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

std::vector<CylindricalShell> Sampling::searchAffordances(const PointCloud::Ptr &cloud,
                                                          const PointCloudRGB::Ptr &cloudrgb, double target_radius)
{
  double start_time = omp_get_wtime();
  double sigma = 2.0 * target_radius;

  // find initial affordances
  std::vector<int> indices = this->affordances.createRandomIndices(cloud, num_init_samples);
  std::vector<CylindricalShell> all_shells = this->affordances.searchAffordances(cloud, indices);

  // visualize
  if (this->is_visualized)
  {
    Eigen::MatrixXd init_samples(3, num_init_samples);
    for (std::size_t i = 0; i < num_init_samples; i++)
    {
      init_samples(0, i) = cloudrgb->points[indices[i]].x;
      init_samples(1, i) = cloudrgb->points[indices[i]].y;
      init_samples(2, i) = cloudrgb->points[indices[i]].z;
    }
    std::vector<CylindricalShell> no_shells;
    no_shells.resize(0);

    SamplingVisualizer visualizer;
    visualizer.createViewerRGB(cloudrgb, no_shells, init_samples, target_radius);
    visualizer.getViewer()->setSize(800, 600);
    visualizer.getViewer()->setCameraPosition(0.255106, -0.903705, -0.538521, 0.255053, -0.902864, -0.537242,
                                              -0.0206334, -0.835517, 0.549076);

    while (!visualizer.getViewer()->wasStopped())
    {
      visualizer.getViewer()->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  int num_rand_samples = prob_rand_samples * num_samples;
  int num_gauss_samples = num_samples - num_rand_samples;

  Eigen::Matrix3d diag_sigma = Eigen::Matrix3d::Zero();
  diag_sigma.diagonal() << sigma, sigma, sigma;
  Eigen::Matrix3d inv_sigma = diag_sigma.inverse();
  double term = 1.0 / sqrt(pow(2.0 * M_PI, 3.0) * pow(sigma, 3.0));

  // create random generator for normal distribution
  boost::mt19937 *rng = new boost::mt19937();
  rng->seed(time(NULL));
  boost::normal_distribution<> distribution(0.0, 1.0);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator(*rng, distribution);
  Eigen::MatrixXd samples(3, num_samples);

  // find affordances using importance sampling
  for (std::size_t i = 0; i < num_iterations; i++)
  {
    double iteration_start_time = omp_get_wtime();

    // draw samples close to affordances (importance sampling)
    if (this->method == SUM) // sum of Gaussians
    {
      for (std::size_t j = 0; j < num_gauss_samples; j++)
      {
        int idx = rand() % all_shells.size();
        samples(0, j) = all_shells[idx].getCentroid()(0) + generator() * sigma;
        samples(1, j) = all_shells[idx].getCentroid()(1) + generator() * sigma;
        samples(2, j) = all_shells[idx].getCentroid()(2) + generator() * sigma;
      }
    }
    else // max of Gaussians
    {
      int j = 0;
      while (j < num_gauss_samples) // draw samples using rejection sampling
      {
        // draw from sum of Gaussians
        int idx = rand() % all_shells.size();
        Eigen::Vector3d x;
        x(0) = all_shells[idx].getCentroid()(0) + generator() * sigma;
        x(1) = all_shells[idx].getCentroid()(1) + generator() * sigma;
        x(2) = all_shells[idx].getCentroid()(2) + generator() * sigma;

        double maxp = 0;
        for (std::size_t k = 0; k < all_shells.size(); k++)
        {
          double p = (x - all_shells[k].getCentroid()).transpose() * (x - all_shells[k].getCentroid());
          p = term * exp((-1.0 / (2.0 * sigma)) * p);
          if (p > maxp)
            maxp = p;
        }

        double p = (x - all_shells[idx].getCentroid()).transpose() * (x - all_shells[idx].getCentroid());
        p = term * exp((-1.0 / (2.0 * sigma)) * p);
        if (p >= maxp)
        {
          samples.col(j) = x;
          j++;
        }
      }
    }

    // draw random samples
    for (int j = num_samples - num_rand_samples; j < num_samples; j++)
    {
      int r = std::rand() % cloud->points.size();
      while (!pcl::isFinite((*cloud)[r])
          || !this->affordances.isPointInWorkspace(cloud->points[r].x, cloud->points[r].y, cloud->points[r].z))
        r = std::rand() % cloud->points.size();
      samples.col(j) = cloud->points[r].getVector3fMap().cast<double>();
    }

    // visualize
    if (is_visualized)
    {
      SamplingVisualizer visualizer;
//      visualizer.createViewer(cloud, all_shells, samples, target_radius);
      visualizer.createViewerRGB(cloudrgb, all_shells, samples.block(0, 0, 3, num_gauss_samples), target_radius);
      visualizer.getViewer()->setSize(800, 600);
      visualizer.getViewer()->setCameraPosition(0.255106, -0.903705, -0.538521, 0.255053, -0.902864, -0.537242,
                                                -0.0206334, -0.835517, 0.549076);
      visualizer.getViewer()->addText(
          "Iteration " + boost::lexical_cast < std::string > (i) + ", shells: " + boost::lexical_cast < std::string
              > (all_shells.size()) + ", num_gauss_samples: " + boost::lexical_cast < std::string
              > (num_gauss_samples) + ", num_rand_samples: " + boost::lexical_cast < std::string > (num_rand_samples),
          10, 20, 16, 1.0, 1.0, 1.0);

      while (!visualizer.getViewer()->wasStopped())
      {
        visualizer.getViewer()->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
      }
    }

    // find affordances
    std::vector<CylindricalShell> shells = this->affordances.searchAffordancesTaubin(cloud, samples);
    all_shells.insert(all_shells.end(), shells.begin(), shells.end());
    printf("ELAPSED TIME (ITERATION %i): %.3f\n", (int)i, omp_get_wtime() - iteration_start_time);
  }

  printf("elapsed time (affordance search): %.3f sec, total # of affordances found: %i\n", omp_get_wtime() - start_time,
         (int)all_shells.size());
  return all_shells;
}

void Sampling::initParams(const ros::NodeHandle& node)
{
  node.param("num_iterations", this->num_iterations, this->NUM_ITERATIONS);
  node.param("num_samples", this->num_samples, this->NUM_SAMPLES);
  node.param("num_init_samples", this->num_init_samples, this->NUM_INIT_SAMPLES);
  node.param("prob_rand_samples", this->prob_rand_samples, this->PROB_RAND_SAMPLES);
  node.param("visualize_steps", this->is_visualized, this->VISUALIZE_STEPS);
  node.param("sampling_method", this->method, this->METHOD);
}
