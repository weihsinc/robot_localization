#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

bool first = true;
std::string world_frame_id, init_frame_id, gpsInit_frame_id, base_frame_id;
geometry_msgs::TransformStamped tf_worldToInit, tf_worldToGPSinit, tf_worldToBase;

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
 if (first)
 {
  tf_worldToInit.header.stamp = msg->header.stamp;
  tf_worldToInit.header.frame_id = world_frame_id;
  tf_worldToInit.child_frame_id = init_frame_id;
  tf_worldToInit.transform.translation.x = msg->pose.pose.position.x;
  tf_worldToInit.transform.translation.y = msg->pose.pose.position.y;
  tf_worldToInit.transform.translation.z = msg->pose.pose.position.z;
  tf_worldToInit.transform.rotation = msg->pose.pose.orientation;

  tf_worldToGPSinit.header.stamp = msg->header.stamp;
  tf_worldToGPSinit.header.frame_id = world_frame_id;
  tf_worldToGPSinit.child_frame_id = gpsInit_frame_id;
  tf_worldToGPSinit.transform.translation.x = msg->pose.pose.position.x;
  tf_worldToGPSinit.transform.translation.y = msg->pose.pose.position.y;
  tf_worldToGPSinit.transform.translation.z = msg->pose.pose.position.z;
  tf_worldToGPSinit.transform.rotation.x = 0;
  tf_worldToGPSinit.transform.rotation.y = 0;
  tf_worldToGPSinit.transform.rotation.z = 0;
  tf_worldToGPSinit.transform.rotation.w = 1;

  first = false;
 }

 tf_worldToBase.header.stamp = msg->header.stamp;
 tf_worldToBase.header.frame_id = world_frame_id;
 tf_worldToBase.child_frame_id = base_frame_id;
 tf_worldToBase.transform.translation.x = msg->pose.pose.position.x;
 tf_worldToBase.transform.translation.y = msg->pose.pose.position.y;
 tf_worldToBase.transform.translation.z = msg->pose.pose.position.z;
 tf_worldToBase.transform.rotation = msg->pose.pose.orientation;
}

int main(int argc, char** argv)
{	
 ros::init(argc, argv, "gt_tf_broadcaster");
 ros::NodeHandle nh;
 ros::NodeHandle priv_nh("~");

 priv_nh.param<std::string>("world_frame_id", world_frame_id, "world");
 priv_nh.param<std::string>("gpsInit_frame_id", gpsInit_frame_id, "gps_init");
 priv_nh.param<std::string>("init_frame_id", init_frame_id, "init");
 priv_nh.param<std::string>("base_frame_id", base_frame_id, "base_link");
  	
 ros::Rate r(50); // publish transforms in 100Hz
 ros::Subscriber sub = nh.subscribe("odom", 1, odometryCallback);
 tf2_ros::StaticTransformBroadcaster s_tf_br;
 tf2_ros::TransformBroadcaster tf_br;

 while(nh.ok()) 
 {
   if (!first)
   {
  	 s_tf_br.sendTransform(tf_worldToInit);
  	 s_tf_br.sendTransform(tf_worldToGPSinit);
  	 tf_br.sendTransform(tf_worldToBase);
   }
   ros::spinOnce();
   r.sleep();
 }

}
