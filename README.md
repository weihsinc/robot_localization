# **robot_localization** #

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

For detailed information please see documentation here: http://wiki.ros.org/robot_localization

## **How to run it?** ##
Launch sensor drivers:
~~~~
roslaunch robot_localization ekf_sensors.launch
~~~~
   * This launches the drivers for sensors that are fused by the filter. (Wheel encoder, IMU and GPS)

Launch EKF nodes:
~~~~
roslaunch robot_localization dual_ekf.launch
~~~~
   * This launches two EKF nodes and tf publishers.
   * /ekf_odom: EKF node for local pose estimation.
   * /ekf_map: EKF node for global pose estimation.

## **Visualization** ##
~~~~
roslaunch robot_localization rviz.launch
~~~~
   * This launches two RVIZ windows for visualizing 3D pose and odometry with respective configuration files.
   * ekf_odom.rviz: RVIZ for local pose estimation. (grid reference frame: *odom*)
   * ekf_utm.rviz: RVIZ for global pose estimation. (grid reference frame: *gps_init_ENU*)

## **Initialization** ##
**NOTE: At launching, the global pose estimation is undetermined until the vehicle start driving for some distance.**

## **Subscription** ##

The **ekf_odom** node subscribes to

   * */imu/data (sensor_msgs/Imu)*: Roll/Pitch/Yaw and Roll/Pitch/Yaw rates measurements from Xsens IMU.
   * */vehicle_state/velocity (nav_msgs/Odometry)*: Vehicle forward velocity measurement from wheel encoder. 

The **ekf_map** node subscribes to

   * */imu/data (sensor_msgs/Imu)*: Roll/Pitch and Roll/Pitch/Yaw rates measurements from Xsens IMU.
   * */vehicle_state/velocity (nav_msgs/Odometry)*: Vehicle forward velocity measurement from wheel encoder.
   * */garmin_gps/odom (nav_msgs/Odometry)*: Absolute position in UTM coordinates from Garmin GPS.

## **Publication** ##

The **ekf_odom** node publishes

   * */odometry/filtered_odom (nav_msgs/Odometry)*: Local pose estimation
   * Transforms from **odom_frame** to **base_link_frame**.

The **ekf_map** node publishes

   * */odometry/filtered_map (nav_msgs/Odometry)*: Global pose estimation
   * Transforms from **map_frame** to **odom_frame**.

## **Parameters** ##

All the parameters for the dual EKF are set up in the **params/dual_ekf.yaml**, please refer to the template file **params/ekf_template.yaml** or see the online documentation http://docs.ros.org/indigo/api/robot_localization/html/ for the definitions and usage on parameters.