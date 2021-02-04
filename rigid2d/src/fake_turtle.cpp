#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

/// TODO review include
/// TODO node comments

static rigid2d::DiffDrive dd;	
static rigid2d::WheelVel wv;

void callback(const geometry_msgs::Twist::ConstPtr & msg)
{
	using namespace rigid2d;
	
	Vector2D v(msg -> linear.x, msg -> linear.y);
	Twist2D tw(v, msg -> angular.z);
	wv.r_vel = (dd.twist2WheelVel(tw)).r_vel;
	wv.l_vel = (dd.twist2WheelVel(tw)).l_vel;
}

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "fake_turtle");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 50);
	ros::Subscriber sub = n.subscribe("cmd_vel", 50, callback);
	
	double wheel_base;
	double wheel_radius;
	std::string left_wheel_joint;
	std::string right_wheel_joint;
	
	n.getParam("wheel_base", wheel_base);
	n.getParam("wheel_radius", wheel_radius);
	n.getParam("left_wheel_joint", left_wheel_joint);
	n.getParam("right_wheel_joint", right_wheel_joint);
	
	// TODO have some initial pose q0?
	dd.setPhysicalParams(wheel_base, wheel_radius);  //FIXME try this with a constructor here
	ros::Rate r(100);
	
	sensor_msgs::JointState js;
	js.name = {right_wheel_joint, left_wheel_joint};
	
	auto current_time = ros::Time::now();
	auto last_time = ros::Time::now();
	
	while(n.ok())
	{	
		ros::spinOnce();  //TODO if here or in odometry
		current_time = ros::Time::now(); 
		double dt = (current_time - last_time).toSec();
		dd.updatePose(dd.getRWheelPhi()+dt*wv.r_vel, dd.getLWheelPhi()+dt*wv.l_vel);  // only care about wheel angles
		js.position = {dd.getRWheelPhi(), dd.getLWheelPhi()};
		js.velocity = {wv.r_vel, wv.l_vel};
		pub.publish(js);
		last_time = current_time;
		r.sleep();
	}
	return 0;
}
