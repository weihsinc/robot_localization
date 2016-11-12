#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

bool first_call = true;
ros::Publisher pub;

void gpsOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
	
	geometry_msgs::PoseWithCovarianceStamped init_pose;
	init_pose.header.stamp = msg->header.stamp;
	init_pose.header.frame_id = msg->header.frame_id;
	init_pose.pose = msg->pose; // copy the entire poseWithCovariance part in the odometry message
	pub.publish(init_pose);
	first_call = false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setInitialPose");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/garmin_gps/odom", 1, gpsOdometryCallback);
	pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 1);
	ros::Rate rate(100);

	while(nh.ok() && first_call) {
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}