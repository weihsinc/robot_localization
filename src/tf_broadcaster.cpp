#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

bool gtcb_first_call = true;
bool gpscb_first_call = true;
geometry_msgs::TransformStamped tf_worldToGTinit, tf_worldToGTodom, tf_worldToGT, tf_worldToGPSinit;

void groundTruthOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
 	
 	if (gtcb_first_call) {
 		tf_worldToGTodom.header.stamp = msg->header.stamp;
 		tf_worldToGTodom.header.frame_id = msg->header.frame_id;
 		tf_worldToGTodom.child_frame_id = "gt_odom";
 		tf_worldToGTodom.transform.translation.x = msg->pose.pose.position.x;
 		tf_worldToGTodom.transform.translation.y = msg->pose.pose.position.y;
 		tf_worldToGTodom.transform.translation.z = msg->pose.pose.position.z;
 		tf_worldToGTodom.transform.rotation = msg->pose.pose.orientation;
		
 		tf_worldToGTinit.header.stamp = msg->header.stamp;
 		tf_worldToGTinit.header.frame_id = msg->header.frame_id;
 		tf_worldToGTinit.child_frame_id = "gt_init_ENU";
 		tf_worldToGTinit.transform.translation.x = msg->pose.pose.position.x;
 		tf_worldToGTinit.transform.translation.y = msg->pose.pose.position.y;
 		tf_worldToGTinit.transform.translation.z = msg->pose.pose.position.z;
 		tf_worldToGTinit.transform.rotation.x = 0;
 		tf_worldToGTinit.transform.rotation.y = 0;
 		tf_worldToGTinit.transform.rotation.z = 0;
 		tf_worldToGTinit.transform.rotation.w = 1;

		gtcb_first_call = false;
	}

 	tf_worldToGT.header.stamp = msg->header.stamp;
 	tf_worldToGT.header.frame_id = msg->header.frame_id;
 	tf_worldToGT.child_frame_id = msg->child_frame_id;
 	tf_worldToGT.transform.translation.x = msg->pose.pose.position.x;
 	tf_worldToGT.transform.translation.y = msg->pose.pose.position.y;
 	tf_worldToGT.transform.translation.z = msg->pose.pose.position.z;
 	tf_worldToGT.transform.rotation = msg->pose.pose.orientation;
}

void gpsOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
 	
 	if (gpscb_first_call) {
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
		gpscb_first_call = false;
	}
}

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle nh;
	ros::Rate r(100); // publish transforms in 100Hz
	ros::Subscriber gt_sub = nh.subscribe("/odom", 1, groundTruthOdometryCallback);
	ros::Subscriber gps_sub = nh.subscribe("/garmin_gps/odom", 1, gpsOdometryCallback);
	tf2_ros::StaticTransformBroadcaster stbr;
	tf2_ros::TransformBroadcaster tbr;

	while(nh.ok()) {
		if (!gtcb_first_call) {
			stbr.sendTransform(tf_worldToGTinit);
			stbr.sendTransform(tf_worldToGTodom);
			tbr.sendTransform(tf_worldToGT);
		}
		if (!gpscb_first_call) {
			stbr.sendTransform(tf_worldToGPSinit);	
		}
		ros::spinOnce();
		r.sleep();
	}
}
