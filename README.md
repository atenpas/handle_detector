## https://github.com/atenpas/handle_detector-release.git (hydro) - 1.0.0-2

User `andreas@andreas-Dell-System-XPS-L502X` released the packages in the `https://github.com/atenpas/handle_detector-release.git` repository into the `hydro` distro by running `/usr/bin/bloom-release --rosdistro hydro --track hydro https://github.com/atenpas/handle_detector-release.git --edit` on `Mon, 10 Mar 2014 03:34:11 -0000`

The `handle_detector` package was released.

Version of package(s) in repository `https://github.com/atenpas/handle_detector-release.git`:
- rosdistro version: `null`
- old version: `1.0.0-1`
- new version: `1.0.0-2`

Versions of tools used:
- bloom version: `0.5.1`
- catkin_pkg version: `0.1.26`
- rosdep version: `0.10.25`
- rosdistro version: `0.3.4`
- vcstools version: `0.1.33`


## https://github.com/atenpas/handle_detector.git (hydro) - 1.0.0-1

User `andreas@andreas-Dell-System-XPS-L502X` released the packages in the `https://github.com/atenpas/handle_detector.git` repository into the `hydro` distro by running `/usr/bin/bloom-release --rosdistro hydro --track hydro https://github.com/atenpas/handle_detector.git --edit` on `Mon, 10 Mar 2014 03:22:06 -0000`

The `handle_detector` package was released.

Version of package(s) in repository `https://github.com/atenpas/handle_detector.git`:
- rosdistro version: `null`
- old version: `1.0.0-0`
- new version: `1.0.0-1`

Versions of tools used:
- bloom version: `0.5.1`
- catkin_pkg version: `0.1.26`
- rosdep version: `0.10.25`
- rosdistro version: `0.3.4`
- vcstools version: `0.1.33`


## https://github.com/atenpas/handle_detector.git (hydro) - 1.0.0-0

User `andreas@andreas-Dell-System-XPS-L502X` released the packages in the `https://github.com/atenpas/handle_detector.git` repository into the `hydro` distro by running `/usr/bin/bloom-release --rosdistro hydro --track hydro https://github.com/atenpas/handle_detector.git --edit` on `Mon, 10 Mar 2014 03:17:11 -0000`

The `handle_detector` package was released.

Version of package(s) in repository `https://github.com/atenpas/handle_detector.git`:
- rosdistro version: `null`
- old version: `null`
- new version: `1.0.0-0`

Versions of tools used:
- bloom version: `0.5.1`
- catkin_pkg version: `0.1.26`
- rosdep version: `0.10.25`
- rosdistro version: `0.3.4`
- vcstools version: `0.1.33`


# Handle Detector

**Author:** Andreas ten Pas (atp@ccs.neu.edu)

**Version:** 1.0

## REQUIREMENTS

1. ROS Hydro (http://wiki.ros.org/hydro)
2. Lapack (install in Ubuntu using: sudo apt-get install liblapack-dev)
3. Openni_launch (http://wiki.ros.org/openni_launch; install in Ubuntu using: sudo apt-get install ros-hydro-openni-launch)


## INSTRUCTIONS

### (A) Installation
1. Clone the repository using *git*.
2. Copy the cloned repository into the 'src' folder of your ROS workspace.
3. Recompile your ROS workspace: $ catkin_make.

### (B) Localize handles in a pcd file
1. Set the parameter 'file' in launch/localization_pcd_file.launch to the absolute path of some 
pcd file (a sample pcd file is provided in /pathToHandleDetectorPackage/data/stagedcleaning8.pcd).
2. Start roscore: $ roscore.
3. Run the handle localization: $ roslaunch handle_detector localization_pcd_file.launch.
4. Use RViz to visualize the results (see also below): $ rosrun rviz rviz.

### (C) Localize handles using a RGB-D camera (openni compatible device required; tested with Asus Xtion Pro)
1. Set-up one or more objects in front of the RGB-D camera, and have the RGB-D camera running.
2. Start roscore: $ roscore.
3. Start openni_launch: $ roslaunch openni_launch openni.launch.
4. Run the handle localization: $ roslaunch handle_detector localization_sensor.launch.
5. Use RViz to visualize the results (see also below): $ rosrun rviz rviz.

### (D) Grasp handles using localization information
1. Follow the steps described in **(C)**.
2. Subscribe to the ROS topic /localization/handle_list to get a list of handles.
3. This list gives the pose, radius, extent, major axis, and normal axis for each affordance 
(cylindrical shell) in every handle. 
4. To see the structure of the messages published here, use: $ rosmsg show handle_detector/...
5. Use the given localization information, to decide for a target handle, and grasp it using your robot.


## COMMANDS

### run handle localization on a *.pcd file
roslaunch handle_detector localization_pcd_file.launch

### run handle localization on range sensor input
roslaunch handle_detector localization_sensor.launch


## VISUALIZATION IN RVIZ

### show input point cloud
add a PointCloud2 and set the topic to: /localization/point_cloud

### show all affordances
add a MarkerArray and set the topic to: /localization/visualization_all_affordances

### show all handles
add a MarkerArray and set the topic to: /localization/visualization_all_handles

### show handle i (i = 0, 1, ...)
add a MarkerArray and set the topic to: /localization/visualization_handle_i


## ROS TOPICS WITH HANDLE AND AFFORDANCE INFORMATION

### /localization/cylinder_list
contains a list of cylinders (affordances) with pose, radius, extent, major axis, and normal axis

### /localization/handle_list
contains a list of handles where each handle consists of a list of cylinders (affordances)

## ROS LAUNCH PARAMETERS

### Affordance Localization

#### file
the location of the *.pcd file on the file system (absolute path)

#### target radius
the radius of the target handle

#### target_radius_error
the error permitted on the target radius

#### affordance_gap
the size of the gap around the affordance

#### sample_size
the number of point neighborhoods to be sampled from the point cloud

#### use_clearance_filter
whether the clearance filter (enough gap around affordance) is used

#### use_occlusion_filter
whether the occlusion filter is used

#### curvature_estimator
the method that is used to estimate curvature (0: Taubin Quadric Fitting, 1: PCA, 2: Normals)

#### point_cloud_source
the source of the point cloud (0: *.pcd file, 1: range sensor)

#### update_interval
the interval in seconds at which the algorithm is repeated

#### Workspace Limits
the minimum and maximum values of the workspace in three dimensions

#### num_threads
the number of threads to use in Taubin Quadric Fitting
		
### Handle Localization

#### ransac_runs
the number of times RANSAC is run to detect handles

#### ransac_min_inliers
the minimum number of colinear affordances in a handle

#### ransac_dist_radius
the axis distance below which two affordances are considered to belong to the same handle

#### ransac_orient_radius
the orientation distance below which two affordances are considered to belong to the same handle

#### ransac_radius_radius
the radius distance below which two affordances are considered to belong to the same handle
