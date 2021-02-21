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
#include <random>


static rigid2d::DiffDrive dd;	
static rigid2d::WheelVel wv;
// Noise parameters
static double vx_noise = 0.01;
static double w_noise = 0.01;

/// \brief Seeds the random number generation once
/// \return a reference to the pseudo-random number generator object
std::mt19937 & get_random()  // function from lecture notes
{
	// static variables inside a function are created once and persist for the remainder of the program
	static std::random_device rd{}; 
	static std::mt19937 mt{rd()};
	// we return a reference to the pseudo-random number genrator object. This is always the
	// same object every time get_random is called
	return mt;
}

/// \brief Updates wheel angular velocities when a Twist message arrives
/// \param msg - a pointer to the geometry_msg/Twist message describing the commanded twist
void callback(const geometry_msgs::Twist::ConstPtr & msg)
{
	using namespace rigid2d;
	static std::normal_distribution<> vx_gauss(0, vx_noise);
	static std::normal_distribution<> w_gauss(0, w_noise);	
	Vector2D v(msg -> linear.x + vx_gauss(get_random()), msg -> linear.y);
	Twist2D tw(v, msg -> angular.z + w_noise(get_random());
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
	dd.setPhysicalParams(wheel_base, wheel_radius);
	n.getParam("left_wheel_joint", left_wheel_joint);
	n.getParam("right_wheel_joint", right_wheel_joint); 
	
	n.getParam("vx_noise", vx_noise);
	n.getParam("w_noise", w_noise);

	double slip_max = 0.5;
	n.getParam("slip_max", slip_max);
	std::uniform_real_distribution<> slip_unif(0, slip_max);
	
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
		dd.updatePose(dd.getLWheelPhi()+dt*wv.l_vel, dd.getRWheelPhi()+dt*wv.r_vel, slip_unif(get_random()), slip_unif(get_random()));
		js.header.stamp = current_time;
		js.position = {dd.getLWheelPhi(), dd.getRWheelPhi()};
		js.velocity = {wv.l_vel, wv.r_vel};
		pub.publish(js);
		last_time = current_time;
		r.sleep();
	}
	return 0;
}
