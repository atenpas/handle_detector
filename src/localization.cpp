#include "affordances.h"
#include "handle_detector/CylinderArrayMsg.h"
#include "handle_detector/CylinderMsg.h"
#include "handle_detector/HandleListMsg.h"
#include <ctype.h>
#include "cylindrical_shell.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include "messages.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <stdlib.h> 
#include <stdio.h>
#include <string.h>
#include <vector>
#include "visualizer.h"
#define EIGEN_DONT_PARALLELIZE

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const std::string RANGE_SENSOR_FRAME = "/camera_rgb_optical_frame";
const std::string RANGE_SENSOR_TOPIC = "/camera/depth_registered/points";
//~ const std::string RANGE_SENSOR_TOPIC = "/head_camera/depth/points"; // simulated Kinect

// input and output ROS topic data
PointCloud::Ptr g_cloud(new PointCloud);
Affordances g_affordances;
std::vector<CylindricalShell> g_cylindrical_shells;
std::vector< std::vector<CylindricalShell> > g_handles2;

// synchronization
double g_prev_time;
double g_update_interval;
bool g_has_read = false;

void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	if (omp_get_wtime() - g_prev_time < g_update_interval)
		return;
		
	// check whether input frame is equivalent to range sensor frame constant
	std::string input_frame = input->header.frame_id;
	if (input_frame.compare(RANGE_SENSOR_FRAME) != 0)
	{
		printf("Input frame %s is not equivalent to output frame %s ! Exiting ...\n", input_frame.c_str(), RANGE_SENSOR_FRAME.c_str());
		std::exit(EXIT_FAILURE);
	}
	printf("input frame: %s, output frame: %s\n", input_frame.c_str(), RANGE_SENSOR_FRAME.c_str());
	
	// convert ROS sensor message to PCL point cloud
	PointCloud::Ptr cloud(new PointCloud);
	pcl::fromROSMsg(*input, *cloud);
	g_has_read = true;
	
	// organize point cloud for Organized Nearest Neighbors Search
	g_cloud->width = 640;
	g_cloud->height = 480;
	g_cloud->points.resize(g_cloud->width * g_cloud->height);
	for (int i = 0; i < g_cloud->height; i++)
	{
		for (int j = 0; j < g_cloud->width; j++)
		{
			g_cloud->points[i * g_cloud->width + j] = cloud->points[i * g_cloud->width + j];
		}
	}
	
	// store data to file
	//~ pcl::PointCloud<pcl::PointXYZRGB>::Ptr stored_cloud;
	//~ pcl::fromROSMsg(*input, *stored_cloud);
	//~ pcl::io::savePCDFileASCII("/home/andreas/test_pcd.pcd", *stored_cloud);
	
  // search grasp affordances
  g_cylindrical_shells = g_affordances.searchAffordances(g_cloud);
  
  // search handles
  g_handles2 = g_affordances.searchHandles(g_cloud, g_cylindrical_shells);
	
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
	int point_cloud_source;
	node.param("point_cloud_source", point_cloud_source, SENSOR);
	printf("point cloud source: %i\n", point_cloud_source);
	
	// set point cloud update interval from launch file 
	node.param("update_interval", g_update_interval, 10.0);
	
	// read parameters
	g_affordances.initParams(node);
	
	std::string range_sensor_frame;
	ros::Subscriber sub;
	
	// point cloud read from file    
  if (point_cloud_source == PCD_FILE)
  {	
		double start_time = omp_get_wtime();
		range_sensor_frame = "/map";
		std::string file = g_affordances.getPCDFile();

		// load point cloud from PCD file
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *g_cloud) == -1)
		{
			std::cerr<<"Couldn't read pcd file"<< std::endl;
			return (-1);
		}
		
		printf("Loaded *.pcd-file: %s\n", file.c_str());
					
		// search grasp affordances
    g_cylindrical_shells = g_affordances.searchAffordances(g_cloud);
    
    // search handles
    g_handles2 = g_affordances.searchHandles(g_cloud, g_cylindrical_shells);
		
		// set boolean variable so that visualization topics get updated
		g_has_read = true;
		
		// measure runtime
		double end_time = omp_get_wtime();
		double elapsed_time = end_time - start_time;
		printf("Affordance and handle search done in %.3f sec.\n", elapsed_time);
	}
	// point cloud read from sensor
	else if (point_cloud_source == SENSOR)
	{		
		printf("Reading point cloud data from sensor topic: %s\n", RANGE_SENSOR_TOPIC.c_str());
		range_sensor_frame = RANGE_SENSOR_FRAME;
		sub = node.subscribe(RANGE_SENSOR_TOPIC, 10, chatterCallback);
	}
	
	// visualization of point cloud, grasp affordances, and handles
	Visualizer visualizer(g_update_interval);
	ros::Publisher cylinder_pub = node.advertise<handle_detector::CylinderArrayMsg>("cylinder_list", 10);
	ros::Publisher handles_pub = node.advertise<handle_detector::HandleListMsg>("handle_list", 10);
	ros::Publisher pcl_pub = node.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
	sensor_msgs::PointCloud2 pc2msg;
	PointCloud::Ptr cloud_vis(new PointCloud);
	std::vector<visualization_msgs::MarkerArray> marker_arrays;
	visualization_msgs::MarkerArray marker_array_msg;
	visualization_msgs::MarkerArray marker_array_msg_handles;
	
	// publication of grasp affordances and handles as ROS topics
	Messages messages;
	ros::Publisher marker_array_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_all_affordances", 10);
	ros::Publisher marker_array_pub_handles = node.advertise<visualization_msgs::MarkerArray>("visualization_all_handles", 10);
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
			cloud_vis = g_affordances.workspaceFilter(g_cloud);
			ROS_INFO("update cloud");
				
			// create cylinder messages for visualization and ROS topic
      marker_array_msg = visualizer.createCylinders(g_cylindrical_shells, range_sensor_frame);
      cylinder_list_msg = messages.createCylinderArray(g_cylindrical_shells, range_sensor_frame);
			ROS_INFO("update visualization");
				
			// create handle messages for visualization and ROS topic
			handle_list_msg = messages.createHandleList(g_handles2, range_sensor_frame);
			visualizer.createHandles(g_handles2, range_sensor_frame, marker_arrays, 
                              marker_array_msg_handles);
			handle_pubs.resize(g_handles2.size());
			for (int i=0; i < handle_pubs.size(); i++)
				handle_pubs[i] = node.advertise<visualization_msgs::MarkerArray>("visualization_handle_" + boost::lexical_cast<std::string>(i), 10);
      ROS_INFO("update messages");
			
			g_has_read = false;
		}
		
		// publish point cloud
		pcl::toROSMsg(*cloud_vis, pc2msg);
		pc2msg.header.stamp = ros::Time::now();
		pc2msg.header.frame_id = range_sensor_frame;
		pcl_pub.publish(pc2msg);
		
		// publish cylinders for visualization
		marker_array_pub.publish(marker_array_msg);
		
		// publish handles for visualization
		for (int i=0; i < handle_pubs.size(); i++)
			handle_pubs[i].publish(marker_arrays[i]);
		
		// publish handles for visualization	
		marker_array_pub_handles.publish(marker_array_msg_handles);
		
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
