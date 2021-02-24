/// \file
/// \brief A kinematic simulation of a differential drive robot subject to sensor noise and wheel slippage, showing the location of obstacles in rviz
///
/// PARAMETERS:
///		wheel_base (double): the distance between the robot wheels
///		wheel_radius (double): the radius of the wheels
///		left_wheel_joint (string): the name of the left wheel joint
///		right_wheel_joint (string): the name of the right wheel joint
///		vx_noise (double): Gaussian noise on the commanded linear twist
///		w_noise (double): Gaussian noise on the commanded rotational twist
///		tube_var (std::vector<double>): covariance matrix of x and y relative sensor noise (x, y, xy)  //TODO add
///		x_tubes (std::vector<double>): vector containing x coordinates of the obstacle tubes
///		y_tubes (std::vector<double>): vector containing y coordinates of the obstacle tubes
///		tube_radius (double): radius of the obstacle tubes
///		d_max_tubes (double): maximum distance beyond which tubes are not visible
/// PUBLISHES:
///     joint_states (sensor_msgs/JointState): angles and angular velocities of left and right robot wheels  //TODO remove?
///		visualization_marker_array (visualization_msgs/MarkerArray): true location of the tubes in the environment
///		real_path (nav_msgs/Path): trajectory of the robot
///		fake_sensor (visualization_msgs/MarkerArray): measured position of the tubes relative to the robot
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): input twist that causes motion of robot wheels

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <string>
#include <random>
#include <vector>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

static rigid2d::DiffDrive dd;	
static rigid2d::WheelVel wv;
// Noise parameters
static double vx_noise = 0.01;
static double w_noise = 0.01;

static std::vector<double> x_tubes;  //TODO move these in main?
static std::vector<double> y_tubes;
static std::vector<double> tube_var = {0.01, 0.01, 0.0};
static double tube_radius = 0.01;
static double d_max_tubes = 1.0;
//TODO covariance matrix for relative x-y sensor

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
	static std::normal_distribution<> vx_gauss(0.0, vx_noise);
	static std::normal_distribution<> w_gauss(0.0, w_noise);	
	Vector2D v(msg -> linear.x + vx_gauss(get_random()), msg -> linear.y);
	Twist2D tw(v, msg -> angular.z + w_gauss(get_random()));
	wv.l_vel = (dd.twist2WheelVel(tw)).l_vel;
	wv.r_vel = (dd.twist2WheelVel(tw)).r_vel;
}

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "tube_world");
	ros::NodeHandle n;
	//ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Publisher pub_tubes = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000, true);
	ros::Publisher pub_path = n.advertise<nav_msgs::Path>("real_path", 1000);
	ros::Publisher pub_sensor = n.advertise<visualization_msgs::MarkerArray>("fake_sensor", 1000);
	tf2_ros::TransformBroadcaster broadcaster;
	
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
	
	n.getParam("x_tubes", x_tubes);
	n.getParam("y_tubes", y_tubes);
	n.getParam("tube_radius", tube_radius);
	n.getParam("d_max_tubes", d_max_tubes);

	double slip_max = 0.5;
	n.getParam("slip_max", slip_max);
	std::uniform_real_distribution<> slip_unif(0, slip_max);
	
	ros::Rate r(10);
	sensor_msgs::JointState js;
	js.name = {left_wheel_joint, right_wheel_joint};
	
	auto current_time = ros::Time::now();
	auto last_time = ros::Time::now();
	
	visualization_msgs::MarkerArray real_marker_arr;
	
	for(std::size_t i = 0; i < x_tubes.size(); ++i)
	{
		visualization_msgs::Marker m;
		m.header.stamp = current_time;
		m.header.frame_id = "world";
		m.ns = "real";
		m.id = i;
		m.type = visualization_msgs::Marker::CYLINDER;
		m.action = visualization_msgs::Marker::ADD;
		m.pose.position.x = x_tubes[i];
		m.pose.position.y = y_tubes[i];
		m.pose.position.z = 0;
		m.pose.orientation.x = 0;
		m.pose.orientation.y = 0;
		m.pose.orientation.z = 0;
		m.pose.orientation.w = 1;
		m.scale.x = tube_radius;
		m.scale.y = tube_radius;
		m.scale.z = 0.2;
		m.color.r = 1.0;
		m.color.b = 0.0;
		m.color.g = 0.0;
		m.color.a = 1.0;
		real_marker_arr.markers.push_back(m);
	}
	pub_tubes.publish(real_marker_arr);
	
	geometry_msgs::TransformStamped trans;
	trans.header.frame_id = "world";
	trans.child_frame_id = "turtle";
	
	nav_msgs::Path path;
	path.header.frame_id = "world";
	visualization_msgs::MarkerArray relative_marker_arr;
	
	
	while(n.ok())
	{	
		ros::spinOnce();
		current_time = ros::Time::now(); 
		double dt = (current_time - last_time).toSec();
		dd.updatePose(dd.getLWheelPhi()+dt*wv.l_vel, dd.getRWheelPhi()+dt*wv.r_vel, slip_unif(get_random()), slip_unif(get_random()));
		//js.header.stamp = current_time;
		//js.position = {dd.getLWheelPhi(), dd.getRWheelPhi()};
		//js.velocity = {wv.l_vel, wv.r_vel};
		//pub.publish(js);
		
		// Publish frame transformation
		trans.header.stamp = current_time;
		trans.transform.translation.x = dd.getX();
		trans.transform.translation.y = dd.getY();
		trans.transform.translation.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, dd.getTheta());
		trans.transform.rotation.x = q.x();
		trans.transform.rotation.y = q.y();
		trans.transform.rotation.z = q.z();
		trans.transform.rotation.w = q.w();
		broadcaster.sendTransform(trans);	
		
		//Publish path
		path.header.stamp = current_time;
		geometry_msgs::PoseStamped pos;
		pos.header.stamp = current_time;
		pos.pose.position.x = dd.getX();
		pos.pose.position.y = dd.getY();
		pos.pose.position.z = 0.0;
		pos.pose.orientation.x = q.x();
		pos.pose.orientation.y = q.y();
		pos.pose.orientation.z = q.z();
		pos.pose.orientation.w = q.w();
		path.poses.push_back(pos);
		pub_path.publish(path);
		
		//Publish relative marker pose  //TODO add covariance noise
		Vector2D robot_pos(dd.getX(), dd.getY());
		Transform2D t(robot_pos, dd.getTheta());
		Transform2D tinv = t.inv();
		for(std::size_t i = 0; i < x_tubes.size(); ++i)
		{
			visualization_msgs::Marker m;
			m.header.stamp = current_time; 
			m.header.frame_id = "turtle";
			m.ns = "relative";
			m.id = i;
			m.type = visualization_msgs::Marker::CYLINDER;
			
			Vector2D p_abs(x_tubes[i], y_tubes[i]);
			Vector2D p_rel = tinv(p_abs);
			double mag = magnitude(p_rel);
			if (mag <= d_max_tubes)
			{
				m.action = visualization_msgs::Marker::MODIFY;
			}
			else
			{
				m.action = visualization_msgs::Marker::DELETE;
			}
			m.pose.position.x = p_rel.x;
			m.pose.position.y = p_rel.y;
			m.pose.position.z = 0;
			m.pose.orientation.x = 0;
			m.pose.orientation.y = 0;
			m.pose.orientation.z = 0;
			m.pose.orientation.w = 1;
			m.scale.x = tube_radius;
			m.scale.y = tube_radius;
			m.scale.z = 0.2;
			m.color.r = 1.0;
			m.color.b = 0.0;
			m.color.g = 0.0;
			m.color.a = 1.0;
			relative_marker_arr.markers.push_back(m);
		}
		pub_sensor.publish(relative_marker_arr);
		
		last_time = current_time;
		r.sleep();
	}
	return 0;
}
