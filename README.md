# **robot_localization** #

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

For detailed information please see documentation here: http://wiki.ros.org/robot_localization


## **Installation** ##
Requires "geographic-msgs". If not installed:
~~~~
sudo apt-get install ros-indigo-geographic-msgs
~~~~

## **How to run it?** ##
Launch sensor drivers:
~~~~
roslaunch robot_localization ekf_sensors.launch
~~~~
   * This launches the drivers for sensors that are fused by the filter. (Wheel encoder, IMU and GPS)

Launch dual EKF:
~~~~
roslaunch robot_localization dual_ekf.launch rviz_odom:=<true/false(default)> rviz_utm:=<true/false(default)>
~~~~
   * This launches two EKF nodes and tf publishers.
   * *ekf_odom*: EKF node for local pose estimation.
   * *ekf_map*: EKF node for global pose estimation.
   * args: rviz_odom (ekf_odom.rviz); rviz_utm (ekf_utm.rviz).

Launch Global EKF:
~~~~
roslaunch robot_localization ekf_global.launch rviz:=<true/false(default)>
~~~~
   * This launches global EKF localization node, *ekf_global*.
   * args: rviz (ekf_odom.rviz).

Launch local EKF:
~~~~
roslaunch robot_localization ekf_local.launch rviz:=<true/false(default)>
~~~~
   * This launches local EKF localization node, *ekf_local*.
   * args: rviz (ekf_utm.rviz).

Visualization during operation:
~~~~
roslaunch robot_localization rviz.launch
~~~~
   * This launches two RVIZ windows for visualizing 3D pose and odometry with respective configuration files.
   * ekf_odom.rviz: RVIZ for local odometry. (grid reference frame: *odom*)
   * ekf_utm.rviz: RVIZ for global odometry. (grid reference frame: *gps_init_ENU*)

## **Initialization** ##
**WARNING: At launching, the global pose estimation is undetermined until the vehicle starts driving for some distance. For details about sensor fusion layout, please refer to the Google slide: [Local and Global Pose Estimation for Yamaha Viking](https://drive.google.com/open?id=1ZupPT3fVijkt8HYYW9_nPaK9ds9az5XWv38_J53h8Eo)**

## **Subscriptions** ##

The **ekf_odom** node subscribes to

   * */imu/data (sensor_msgs/Imu)*: Roll/Pitch/Yaw and Roll/Pitch/Yaw rates measurements from Xsens IMU.
   * */vehicle_state/velocity (nav_msgs/Odometry)*: Vehicle forward velocity measurement from wheel encoder. 

The **ekf_map** node subscribes to

   * */imu/data (sensor_msgs/Imu)*: Roll/Pitch (NOTE: Not using IMU's yaw measurement in global EKF node) and Roll/Pitch/Yaw rates measurements from Xsens IMU.
   * */vehicle_state/velocity (nav_msgs/Odometry)*: Vehicle forward velocity measurement from wheel encoder.
   * */garmin_gps/odom (nav_msgs/Odometry)*: Absolute position in UTM coordinates from Garmin GPS.

## **Publications** ##

The **ekf_odom** node publishes

   * */odometry/filtered_odom (nav_msgs/Odometry)*: Local pose estimation
   * */tf (geometry_msgs/TransformStamped)*: Transforms from **odom_frame** to **base_link_frame**.

The **ekf_map** node publishes

   * */odometry/filtered_map (nav_msgs/Odometry)*: Global pose estimation
   * */tf (geometry_msgs/TransformStamped)*: Transforms from **map_frame** to **odom_frame**.

## **Parameters** ##

All the parameters for the dual EKF are set up in the **params/dual_ekf.yaml**, please refer to the template file **params/ekf_template.yaml** or see the online documentation http://docs.ros.org/indigo/api/robot_localization/html/ for the definitions and usage on parameters.

## **Debugging** ##

To debug the EKF pose on live test, use the following command
~~~~
roslaunch robot_localization debug.launch
~~~~
   * This launches the rpy_publisher and the rqt_gui console with rqt_plot plugins for visualizing the time series of live message data.

## **Data Logging** ##
To log a ros bag for EKF, use the launch file **launch/ekf_log.launch**. The launch file has already included the default topics needed, specify the path and file prefix in the "args" tag before recording a bag and use the following command 
~~~~
roslaunch robot_localization ekf_log.launch
~~~~