#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

/// TODO review include
/// TODO node comments

static DiffDrive dd;  // TODO how to access this from callback without initially constructing
// TODO may be able to move these in main()
static double wheel_base;
static double wheel_radius;
static std::string left_wheel_joint;
static std::string right_wheel_joint;	
//
static WheelVel wv;

void callback(const geometry_msgs::Twist::ConstPtr & msg)
{
	using namespace rigid2d;
	
	Vector2D v(msg -> linear.x, msg -> linear.y);
	Twist2D tw(v, msg -> angular.z);
	wv.r_vel = (dd.Twist2WheelVel(tw)).r_vel;
	wv.l_vel = (dd.Twist2WheelVel(tw)).l_vel;	
}

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "fake_turtle");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 50);
	ros::Subscriber sub = n.subscribe("turtle1/cmd_vel", 50, callback);
	
	nh.getParam("wheel_base", wheel_base);
	nh.getParam("wheel_radius", wheel_radius);
	nh.getParam("left_wheel_joint", left_wheel_joint);
	nh.getParam("right_wheel_joint", right_wheel_joint);
	
	// TODO have some initial pose q0
	DiffDrive dd(q0, wheel_base, wheel_radius);
	ros::Rate r(100);
	
	sensor_msgs::JointState js;
	js.name = {right_wheel_joint, left_wheel_joint};
	
	while(n.ok())
	{
		ros::spinOnce(); 
		js.position = {dd.getRWheelPhi(), dd.getLWheelPhi()};
		js.velocity = {wv.r_vel, wv.l_vel};
		pub.publish(js);
		r.sleep();
	}
	return 0;
}
