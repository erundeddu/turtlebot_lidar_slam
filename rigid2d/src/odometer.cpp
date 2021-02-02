#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

/// TODO review include
/// TODO node comments

static rigid2d::DiffDrive dd;
static nav_msgs::Odometry odom;
static geometry_msgs::TransformStamped odom_trans;

void callback(const sensor_msgs::JointState::ConstPtr & msg)
{
	static ros::NodeHandle nh;
	static ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	static tf::TransformBroadcaster broadcaster;
	
	auto current_time = ros::Time::now();
	
	// update position in robot
	double r_phi_wheel_new = msg -> position[0];
	double l_phi_wheel_new = msg -> position[1];
	dd.updatePose(r_phi_wheel_new, l_phi_wheel_new);

	// start referencing http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom here (Access: 2/1/2021)	
	odom.header.stamp = current_time;
	odom.pose.pose.position.x = dd.getX();
	odom.pose.pose.position.y = dd.getY();
	odom.pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(dd.getTheta());
	odom.pose.pose.orientation = odom_quat;
	pub.publish(odom);

	odom_trans.header.stamp = current_time;
	odom_trans.transform.translation.x = dd.getX();
	odom_trans.transform.translation.y = dd.getY();
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	broadcaster.sendTransform(odom_trans);
	
	// end referencing http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom here (Access: 2/1/2021)
}

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "odometer");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joint_states", 50, callback);
	
	double wheel_base;
	double wheel_radius;
	std::string odom_frame_id;
	std::string body_frame_id;
	std::string left_wheel_joint;
	std::string right_wheel_joint;
	
	n.getParam("wheel_base", wheel_base);
	n.getParam("wheel_radius", wheel_radius);
	n.getParam("odom_frame_id", odom_frame_id);
	n.getParam("body_frame_id", body_frame_id);
	n.getParam("left_wheel_joint", left_wheel_joint);
	n.getParam("right_wheel_joint", right_wheel_joint);
	
	odom.header.frame_id = odom_frame_id;
	odom.child_frame_id = body_frame_id;
	odom_trans.header.frame_id = odom_frame_id;
	odom_trans.child_frame_id = body_frame_id;
	
	// TODO have some initial pose q0?
	dd.setPhysicalParams(wheel_base, wheel_radius);
	ros::Rate r(100);
	
	while(n.ok())
	{
		ros::spinOnce(); 
		r.sleep();
	}
	
	return 0;
}
