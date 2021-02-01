#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include "rigid2d/rigid2d.hpp"

/// TODO node comment

void callback(const sensor_msgs::JointState::ConstPtr & msg)
{
	x = msg->x;
	y = msg->y;
	theta = msg->theta;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometer");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Subscriber sub = n.subscribe("sensor_msgs/JointState", 50, callback);
	tf::TransformBroadcaster odom_broadcaster;
	
	double wheel_base;
	double wheel_radius;
	std::string odom_frame_id;
	std::string body_frame_id;
	std::string left_wheel_joint;
	std::string right_wheel_joint;
	
	nh.getParam("wheel_base", wheel_base);
	nh.getParam("wheel_radius", wheel_radius);
	nh.getParam("~odom_frame_id", odom_frame_id);
	nh.getParam("~body_frame_id", body_frame_id);
	nh.getParam("~left_wheel_joint", left_wheel_joint);
	nh.getParam("~right_wheel_joint", right_wheel_joint);
	
	return 0;
}
