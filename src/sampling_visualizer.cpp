#include "handle_detector/sampling_visualizer.h"

pcl::ModelCoefficients SamplingVisualizer::createCylinder(Eigen::Vector3d pt_on_axis, Eigen::Vector3d axis_direction,
                                                          double radius, double extent)
{
  pcl::ModelCoefficients cylinder_coeff;
  cylinder_coeff.values.resize(7);
  cylinder_coeff.values[0] = pt_on_axis.x();
  cylinder_coeff.values[1] = pt_on_axis.y();
  cylinder_coeff.values[2] = pt_on_axis.z();
  cylinder_coeff.values[3] = axis_direction.x() * extent;
  cylinder_coeff.values[4] = axis_direction.y() * extent;
  cylinder_coeff.values[5] = axis_direction.z() * extent;
  cylinder_coeff.values[6] = radius;
  return cylinder_coeff;
}

void SamplingVisualizer::addCylinders(const std::vector<CylindricalShell> &shells, void* viewer_void,
                                      std::string handle_index, double r, double g, double b)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<
      pcl::visualization::PCLVisualizer> *>(viewer_void);

  for (std::size_t i = 0; i < shells.size(); i++)
  {
    std::string id = "cylinder" + handle_index + boost::lexical_cast < std::string > (i);
    viewer->addCylinder(
        createCylinder(shells[i].getCentroid(), shells[i].getCurvatureAxis(), shells[i].getRadius(),
                       shells[i].getExtent()),
        id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, id);
  }
}

void SamplingVisualizer::createViewer(PointCloud::ConstPtr cloud, std::vector<CylindricalShell> shells,
                                      Eigen::MatrixXd samples, double target_radius)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud < pcl::PointXYZ > (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, "sample cloud");

  addCylinders(shells, (void*)&viewer, "", 0, 1.0, 1.0);

  for (std::size_t i = 0; i < samples.cols(); i++)
  {
    pcl::PointXYZ center;
    center.x = samples(0, i);
    center.y = samples(1, i);
    center.z = samples(2, i);
    std::string id = "sphere" + boost::lexical_cast < std::string > (i);
    viewer->addSphere(center, target_radius * 0.6, id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, id);
  }

  this->viewer = viewer;
}

void SamplingVisualizer::createViewerRGB(PointCloudRGB::ConstPtr cloud, std::vector<CylindricalShell> shells,
                                         Eigen::MatrixXd samples, double target_radius)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud < pcl::PointXYZRGB > (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

  addCylinders(shells, (void*)&viewer, "", 0, 1.0, 1.0);

  for (std::size_t i = 0; i < samples.cols(); i++)
  {
    pcl::PointXYZ center;
    center.x = samples(0, i);
    center.y = samples(1, i);
    center.z = samples(2, i);
    std::string id = "sphere" + boost::lexical_cast < std::string > (i);
    viewer->addSphere(center, target_radius * 0.4, id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, id);
  }

  this->viewer = viewer;
}

