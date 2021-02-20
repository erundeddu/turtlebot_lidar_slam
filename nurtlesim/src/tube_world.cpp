/// \file
/// \brief A kinematic simulation of a differential drive robot
///
/// PARAMETERS:
///		wheel_base (double): the distance between the robot wheels
///		wheel_radius (double): the radius of the wheels
///		left_wheel_joint (string): the name of the left wheel joint
///		right_wheel_joint (string): the name of the right wheel joint
/// PUBLISHES:
///     joint_states (sensor_msgs/JointState): angles and angular velocities of left and right robot wheels
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): input twist that causes motion of robot wheels

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"


static rigid2d::DiffDrive dd;	
static rigid2d::WheelVel wv;

/// \brief Updates wheel angular velocities when a Twist message arrives
/// \param msg - a pointer to the geometry_msg/Twist message describing the commanded twist
void callback(const geometry_msgs::Twist::ConstPtr & msg)
{
	using namespace rigid2d;
	
	Vector2D v(msg -> linear.x, msg -> linear.y);
	Twist2D tw(v, msg -> angular.z);
	wv.l_vel = (dd.twist2WheelVel(tw)).l_vel;
	wv.r_vel = (dd.twist2WheelVel(tw)).r_vel;
}

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "tube_world");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
	
	double wheel_base;
	double wheel_radius;
	std::string left_wheel_joint;
	std::string right_wheel_joint;
	
	n.getParam("wheel_base", wheel_base);
	n.getParam("wheel_radius", wheel_radius);
	n.getParam("left_wheel_joint", left_wheel_joint);
	n.getParam("right_wheel_joint", right_wheel_joint);
	
	dd.setPhysicalParams(wheel_base, wheel_radius); 
	ros::Rate r(100);
	
	sensor_msgs::JointState js;
	js.name = {left_wheel_joint, right_wheel_joint};
	
	auto current_time = ros::Time::now();
	auto last_time = ros::Time::now();
	
	while(n.ok())
	{	
		ros::spinOnce();
		current_time = ros::Time::now(); 
		double dt = (current_time - last_time).toSec();
		dd.updatePose(dd.getLWheelPhi()+dt*wv.l_vel, dd.getRWheelPhi()+dt*wv.r_vel);  // only care about wheel angles
		js.header.stamp = current_time;
		js.position = {dd.getLWheelPhi(), dd.getRWheelPhi()};
		js.velocity = {wv.l_vel, wv.r_vel};
		pub.publish(js);
		last_time = current_time;
		r.sleep();
	}
	return 0;
}
