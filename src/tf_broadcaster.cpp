#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

bool first_call = true;
geometry_msgs::TransformStamped tf_worldToGPSinit;

void gpsOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
 	
 	if (first_call) {
 		tf_worldToGPSinit.header.stamp = msg->header.stamp;
 		tf_worldToGPSinit.header.frame_id = msg->header.frame_id;
 		tf_worldToGPSinit.child_frame_id = "gps_init_ENU";
 		tf_worldToGPSinit.transform.translation.x = msg->pose.pose.position.x;
 		tf_worldToGPSinit.transform.translation.y = msg->pose.pose.position.y;
 		tf_worldToGPSinit.transform.translation.z = msg->pose.pose.position.z;
 		tf_worldToGPSinit.transform.rotation.x = 0;
 		tf_worldToGPSinit.transform.rotation.y = 0;
 		tf_worldToGPSinit.transform.rotation.z = 0;
 		tf_worldToGPSinit.transform.rotation.w = 1;
		first_call = false;
	}
}

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle nh;
	ros::Rate r(100); // publish transforms in 100Hz
	ros::Subscriber gps_sub = nh.subscribe("/odom", 1, gpsOdometryCallback);
	tf2_ros::StaticTransformBroadcaster br;

	while(nh.ok()) {
		if (!first_call) {
			br.sendTransform(tf_worldToGPSinit);	
		}
		ros::spinOnce();
		r.sleep();
	}
}
