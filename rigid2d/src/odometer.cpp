#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

/// TODO review include
/// TODO node comments

static ros::Time current_time, last_time;
static DiffDrive dd;  // TODO how to access this from callback without initially constructing
static double wheel_base;
static double wheel_radius;
static std::string odom_frame_id;
static std::string body_frame_id;
static std::string left_wheel_joint;
static std::string right_wheel_joint;
static ros::Publisher pub;
	

void callback(const sensor_msgs::JointState::ConstPtr & msg)
{
	current_time = ros::Time::now();
	
	// update position in robot
	double r_phi_wheel_new = msg -> position[0];
	double l_phi_wheel_new = msg -> position[1];
	dd.updatePose(r_phi_wheel_new, l_phi_wheel_new);

	// start referencing http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom here (Access: 2/1/2021)	
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time
	odom.header.frame_id = odom_frame_id;
	odom.child_frame_id = body_frame_id;
	odom.pose.pose.position.x = dd.getX();
	odom.pose.pose.position.y = dd.getY();
	odom.pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(dd.getTheta());
	odom.pose.pose.orientation = odom_quat;
	odom_pub.publish(odom);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time
	odom_trans.header.frame_id = odom_frame_id;
	odom_trans.child_frame_id = body_frame_id;
	odom_trans.transform.translation.x = dd.getX();
	odom_trans.transform.translation.y = dd.getY();
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);
	
	last_time = current_time;
	// end referencing http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom here (Access: 2/1/2021)
}

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "odometer");
	ros::NodeHandle n;
	pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Subscriber sub = n.subscribe("joint_states", 50, callback);
	tf::TransformBroadcaster odom_broadcaster;
	
	nh.getParam("wheel_base", wheel_base);
	nh.getParam("wheel_radius", wheel_radius);
	nh.getParam("odom_frame_id", odom_frame_id);
	nh.getParam("body_frame_id", body_frame_id);
	nh.getParam("left_wheel_joint", left_wheel_joint);
	nh.getParam("right_wheel_joint", right_wheel_joint);
	
	RobotPose q0;  // initial robot pose
	DiffDrive dd(q0, wheel_base, wheel_radius);
	ros::Rate r(100);
	
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	while(n.ok())
	{
		ros::spinOnce(); 
		r.sleep();
	}
	return 0;
}
