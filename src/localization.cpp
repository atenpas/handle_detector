#include "handle_detector/affordances.h"
#include "handle_detector/CylinderArrayMsg.h"
#include "handle_detector/CylinderMsg.h"
#include "handle_detector/HandleListMsg.h"
#include <ctype.h>
#include "handle_detector/cylindrical_shell.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include "handle_detector/messages.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <stdlib.h> 
#include <stdio.h>
#include <string.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>
#include "handle_detector/visualizer.h"
#define EIGEN_DONT_PARALLELIZE

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

const std::string RANGE_SENSOR_TOPIC = "/camera/depth_registered/points";

// input and output ROS topic data
std::string g_sensor_frame;
PointCloud::Ptr g_cloud(new PointCloud);
Affordances g_affordances;
std::vector<CylindricalShell> g_cylindrical_shells;
std::vector<std::vector<CylindricalShell> > g_handles;
tf::StampedTransform g_transform;

// synchronization
double g_prev_time;
double g_update_interval;
bool g_has_read = false;


void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (omp_get_wtime() - g_prev_time < g_update_interval)
    return;

  // convert ROS sensor message to PCL point cloud
  pcl::fromROSMsg(*input, *g_cloud);
  g_has_read = true;
  g_sensor_frame = input->header.frame_id;

  // search grasp affordances
  double start_time = omp_get_wtime();
  g_cylindrical_shells = g_affordances.searchAffordances(g_cloud, &g_transform);
  if (g_cylindrical_shells.size() == 0)
  {
    printf("No handles found!\n");
    g_prev_time = omp_get_wtime();
    return;
  }

  // search handles
  g_handles = g_affordances.searchHandles(g_cloud, g_cylindrical_shells);

  // measure runtime
  printf("Affordance and handle search done in %.3f sec.\n", omp_get_wtime() - start_time);

  // store current time
  g_prev_time = omp_get_wtime();
}


int main(int argc, char** argv)
{
  // constants
  const int PCD_FILE = 0;
  const int SENSOR = 1;

  // initialize random seed
  srand (time(NULL));

  // initialize ROS
  ros::init(argc, argv, "localization");
  ros::NodeHandle node("~");

  // set point cloud source from launch file
  std::string cloud_topic;
  node.param("cloud_topic", cloud_topic, RANGE_SENSOR_TOPIC);

  // set point cloud update interval from launch file
  node.param("update_interval", g_update_interval, 10.0);

  // read parameters
  g_affordances.initParams(node);

  ros::Subscriber sub;

  g_transform.setIdentity();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vis(new pcl::PointCloud<pcl::PointXYZRGB>);

  // point cloud read from file
  if (!g_affordances.getPCDFile().empty())
  {
    g_sensor_frame = "/map";
    std::string file = g_affordances.getPCDFile();

    // load point cloud from PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file, *g_cloud) == -1)
    {
      std::cerr << "Couldn't read pcd file" << std::endl;
      return (-1);
    }
    printf("Loaded *.pcd-file: %s\n", file.c_str());

    pcl::copyPointCloud(*g_cloud, *cloud_vis);

    //~ // search grasp affordances using indices
    //~ g_cylindrical_shells = g_affordances.searchAffordances(g_cloud);

    // search grasp affordances using samples
    double start_time = omp_get_wtime();
    std::vector<int> indices = g_affordances.createRandomIndices(g_cloud, g_affordances.getNumSamples());
    g_cylindrical_shells = g_affordances.searchAffordances(g_cloud, indices);

    // search handles
    g_handles = g_affordances.searchHandles(g_cloud, g_cylindrical_shells);

    // set boolean variable so that visualization topics get updated
    g_has_read = true;

    // measure runtime
    printf("Affordance and handle search done in %.3f sec.\n", omp_get_wtime() - start_time);
  }
  // point cloud read from sensor
  else if (!cloud_topic.empty())
  {
    // create subscriber for camera topic
    printf("Reading point cloud data from sensor topic: %s\n", cloud_topic.c_str());
    sub = node.subscribe(cloud_topic, 10, chatterCallback);
  }

  // visualization of point cloud, grasp affordances, and handles
  Visualizer visualizer(g_update_interval);
  sensor_msgs::PointCloud2 pc2msg;
  ros::Publisher marker_array_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_all_affordances",
                                                                                    1);
  ros::Publisher marker_array_pub_handles = node.advertise<visualization_msgs::MarkerArray>("visualization_all_handles",
                                                                                            1);
  ros::Publisher marker_array_pub_handle_numbers = node.advertise<visualization_msgs::MarkerArray>(
      "visualization_handle_numbers", 1);
  std::vector<visualization_msgs::MarkerArray> marker_arrays;
  visualization_msgs::MarkerArray marker_array_msg;
  visualization_msgs::MarkerArray marker_array_msg_handles;
  visualization_msgs::MarkerArray marker_array_msg_handle_numbers;

  // publication of grasp affordances and handles as ROS topics
  Messages messages;
  ros::Publisher cylinder_pub = node.advertise<handle_detector::CylinderArrayMsg>("cylinder_list", 1);
  ros::Publisher handles_pub = node.advertise<handle_detector::HandleListMsg>("handle_list", 1);
  ros::Publisher pcl_pub = node.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
  std::vector<ros::Publisher> handle_pubs;
  handle_detector::CylinderArrayMsg cylinder_list_msg;
  handle_detector::HandleListMsg handle_list_msg;

  // how often things are published
  ros::Rate rate(10);

  double prev_time = omp_get_wtime();

  while (ros::ok())
  {
    if (g_has_read)
    {
      // create visual point cloud
//      cloud_vis = g_affordances.workspaceFilter(g_cloud, &g_transform);
      ROS_INFO("update cloud");

      // create cylinder messages for visualization and ROS topic
      marker_array_msg = visualizer.createCylinders(g_cylindrical_shells, g_sensor_frame);
      cylinder_list_msg = messages.createCylinderArray(g_cylindrical_shells, g_sensor_frame);
      ROS_INFO("update visualization");

      // create handle messages for visualization and ROS topic
      handle_list_msg = messages.createHandleList(g_handles, g_sensor_frame);
      visualizer.createHandles(g_handles, g_sensor_frame, marker_arrays, marker_array_msg_handles);
      handle_pubs.resize(g_handles.size());
      for (std::size_t i = 0; i < handle_pubs.size(); i++)
        handle_pubs[i] = node.advertise<visualization_msgs::MarkerArray>(
            "visualization_handle_" + boost::lexical_cast < std::string > (i), 10);

      marker_array_msg_handle_numbers = visualizer.createHandleNumbers(g_handles, g_sensor_frame);

      ROS_INFO("update messages");

      g_has_read = false;
    }

    // publish point cloud
    pcl::toROSMsg(*cloud_vis, pc2msg);
    pc2msg.header.stamp = ros::Time::now();
    pc2msg.header.frame_id = g_sensor_frame;
    pcl_pub.publish(pc2msg);

    // publish cylinders for visualization
    marker_array_pub.publish(marker_array_msg);

    // publish handles for visualization
    for (std::size_t i = 0; i < handle_pubs.size(); i++)
      handle_pubs[i].publish(marker_arrays[i]);

    // publish handles for visualization
    marker_array_pub_handles.publish(marker_array_msg_handles);

    // publish handle numbers for visualization
    marker_array_pub_handle_numbers.publish(marker_array_msg_handle_numbers);

    // publish cylinders as ROS topic
    cylinder_pub.publish(cylinder_list_msg);

    // publish handles as ROS topic
    handles_pub.publish(handle_list_msg);

    //~ ROS_INFO("published %i grasp affordances for grasping", (int) cylinder_list_msg.cylinders.size());
    //~ ROS_INFO("published %i handles for grasping", (int) handle_list_msg.handles.size());
    //~ for(int i=0; i < handle_list_msg.handles.size(); i++)
    //~ std::cout<<" - handle "<<i<<": "<<handle_list_msg.handles[i].cylinders.size()<<std::endl;
    //~ ROS_INFO("published %i cylinders for visualization", (int) marker_array_msg.markers.size());
    //~ ROS_INFO("published %i handles for visualization", (int) handle_pubs.size());
    //~ for(int i=0; i < marker_arrays.size(); i++)
    //~ std::cout<<" - visual handle "<<i<<": "<<marker_arrays[i].markers.size()<<std::endl;

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
