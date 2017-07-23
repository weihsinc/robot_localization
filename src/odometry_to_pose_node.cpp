#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class odometryToPoseConverter
{
public:
	odometryToPoseConverter()
	{
		pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
		sub_ = nh_.subscribe("odom", 1, &odometryToPoseConverter::odometryCallback, this);
	}

	void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		geometry_msgs::PoseStamped pose_msg;
		pose_msg.header = msg->header;
		pose_msg.pose = msg->pose.pose;
		pub_.publish(pose_msg);
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_to_pose");
	
	odometryToPoseConverter odom_to_pose;

	ros::spin();

	return 0;
}