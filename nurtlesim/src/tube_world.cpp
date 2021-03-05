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
///		tube_var (std::vector<double>): covariance matrix of x and y relative sensor noise (x, y, xy)
///		x_tubes (std::vector<double>): vector containing x coordinates of the obstacle tubes
///		y_tubes (std::vector<double>): vector containing y coordinates of the obstacle tubes
///		tube_radius (double): radius of the obstacle tubes
///		d_max_tubes (double): maximum distance beyond which tubes are not visible
///		robot_radius (double): collision radius of the robot
///		lidar_min_range (double): minimum object to lidar distance that is sensed
///		lidar_max_range (double): maximum object to lidar distance that is sensed
///		lidar_dtheta (double): angle increment of lidar scan in degrees
///		lidar_n_samples (int): number of samples per lidar scan
///		lidar_resolution (double): resolution of lidar distance measurements
///		lidar_noise (double): Gaussian noise on the lidar distance measurements
///		border_width (double): width of the simulated border
///		border_height (double): height of the simulated border
/// PUBLISHES:
///		visualization_marker_array (visualization_msgs/MarkerArray): true location of the tubes in the environment
///		real_path (nav_msgs/Path): trajectory of the robot
///		fake_sensor (visualization_msgs/MarkerArray): measured position of the tubes relative to the robot
///     joint_states (sensor_msgs/JointState): angles and angular velocities of left and right robot wheels
///		laser_sim (sensor_msgs/LaserScan): simulated lidar data
///		borders (visualization_msgs/Marker): simulated border lines
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
#include <sensor_msgs/LaserScan.h>
#include <armadillo>
#include <string>
#include <random>
#include <vector>
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nurtlesim/multivariate.hpp"
#include "nurtlesim/circles.hpp"

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
	// define distributions
	static std::normal_distribution<> vx_gauss(0.0, vx_noise);
	static std::normal_distribution<> w_gauss(0.0, w_noise);
	// add noise to the linear twist
	Vector2D v(msg -> linear.x + vx_gauss(get_random()), msg -> linear.y);
	// add noise to the rotational twist
	Twist2D tw(v, msg -> angular.z + w_gauss(get_random()));
	// convert twist to wheel speeds
	wv.l_vel = (dd.twist2WheelVel(tw)).l_vel;
	wv.r_vel = (dd.twist2WheelVel(tw)).r_vel;
}



int main(int argc, char** argv)
{
	using namespace rigid2d;
	using namespace circles;
	
	ros::init(argc, argv, "tube_world");
	ros::NodeHandle n;
	// define publishers and subscribers
	ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Publisher pub_tubes = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000, true);
	ros::Publisher pub_path = n.advertise<nav_msgs::Path>("real_path", 1000);
	ros::Publisher pub_sensor = n.advertise<visualization_msgs::MarkerArray>("fake_sensor", 1000);
	ros::Publisher pub_laser = n.advertise<sensor_msgs::LaserScan>("laser_sim", 1000);
	ros::Publisher pub_borders = n.advertise<visualization_msgs::Marker>("borders", 1000, true);
	tf2_ros::TransformBroadcaster broadcaster;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
	
	// define variables to store parameters and retrieve parameters
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
	
	std::vector<double> x_tubes;
	std::vector<double> y_tubes;
	std::vector<double> tube_var = {0.01, 0.01, 0.0};
	double tube_radius = 0.01;
	double d_max_tubes = 1.0;
	double slip_max = 0.5;
	double robot_radius = 0.1;
	double lidar_min_range = 0.12;
	double lidar_max_range = 3.5;
	double lidar_dtheta = 1.0;
	int lidar_n_samples = 360;
	double lidar_resolution = 0.015;
	double lidar_noise = 0.01;
	double border_width = 4.0;
	double border_height = 4.0;
	n.getParam("x_tubes", x_tubes);
	n.getParam("y_tubes", y_tubes);
	n.getParam("tube_var", tube_var);
	n.getParam("tube_radius", tube_radius);
	n.getParam("d_max_tubes", d_max_tubes);
	n.getParam("slip_max", slip_max);
	n.getParam("robot_radius", robot_radius);
	n.getParam("lidar_min_range", lidar_min_range);
	n.getParam("lidar_max_range", lidar_max_range);
	n.getParam("lidar_dtheta", lidar_dtheta);
	n.getParam("lidar_n_samples", lidar_n_samples);
	n.getParam("lidar_resolution", lidar_resolution);
	n.getParam("lidar_noise", lidar_noise);
	n.getParam("border_width", border_width);
	n.getParam("border_height", border_height);
	
	// critical distance for collision
	double d_crit = tube_radius + robot_radius;
	
	// define gaussian distributions given ros parameters
	std::uniform_real_distribution<> slip_unif(0, slip_max);
	
	// obtain matrix for x-y sensor noise bivariate Gaussian
	arma::Mat<double> Q = { {tube_var[0], tube_var[2]},
							{tube_var[2], tube_var[1]} };
	Multivar mv_xy(Q);
	
	ros::Rate r(100);
	sensor_msgs::JointState js;
	js.name = {left_wheel_joint, right_wheel_joint};
	
	auto current_time = ros::Time::now();
	auto last_time = ros::Time::now();
	
	// define message to publish transformation between world frame and turtle frame
	geometry_msgs::TransformStamped trans;
	trans.header.frame_id = "world";
	trans.child_frame_id = "turtle";
	
	// define message to publish path of turtle frame
	nav_msgs::Path path;
	path.header.frame_id = "world";
	
	// contains true position of the markers, expressed in terms of the world frame
	visualization_msgs::MarkerArray real_marker_arr;
	for(std::size_t i = 0; i < x_tubes.size(); ++i)
	{
		visualization_msgs::Marker m;  // initialize marker to add to the array
		m.header.stamp = current_time;
		m.header.frame_id = "world";  // relative to the world (fixed) frame
		m.ns = "real";
		m.id = i+1;  // unique id under namespace "real"
		m.type = visualization_msgs::Marker::CYLINDER;
		m.action = visualization_msgs::Marker::ADD;
		// assign x and y positions of the marker as specified by the ros parameter
		m.pose.position.x = x_tubes[i]; 
		m.pose.position.y = y_tubes[i];
		m.pose.position.z = 0;
		m.pose.orientation.x = 0;
		m.pose.orientation.y = 0;
		m.pose.orientation.z = 0;
		m.pose.orientation.w = 1;
		// scale rviz visualization with tube_radius
		m.scale.x = tube_radius;
		m.scale.y = tube_radius;
		m.scale.z = 0.2;
		// red marker, not transparent
		m.color.r = 0.0;
		m.color.b = 0.0;
		m.color.g = 1.0;
		m.color.a = 1.0;
		real_marker_arr.markers.push_back(m);  // add marker to the array
	}
	pub_tubes.publish(real_marker_arr);  // publish once MarkerArray
	
	// visualization for simulated borders
	visualization_msgs::Marker brd;
	brd.header.stamp = current_time;
	brd.header.frame_id = "world";
	brd.ns = "lines";
	brd.id = 1;
	brd.type = visualization_msgs::Marker::LINE_STRIP;
	brd.action = visualization_msgs::Marker::ADD;
	brd.pose.orientation.x = 0.0;
	brd.pose.orientation.y = 0.0;
	brd.pose.orientation.z = 0.0;
	brd.pose.orientation.w = 1.0;
	brd.scale.x = 0.05;
	brd.color.r = 0.0;
	brd.color.g = 0.0;
	brd.color.b = 0.0;
	brd.color.a = 1.0;
	geometry_msgs::Point line_pt;
	line_pt.x = -border_width/2;
	line_pt.y = -border_height/2;
	line_pt.z = 0.0;
	brd.points.push_back(line_pt);
	line_pt.x = border_width/2;
	line_pt.y = -border_height/2;
	line_pt.z = 0.0;		
	brd.points.push_back(line_pt);
	line_pt.x = border_width/2;
	line_pt.y = border_height/2;
	line_pt.z = 0.0;
	brd.points.push_back(line_pt);
	line_pt.x = -border_width/2;
	line_pt.y = border_height/2;
	line_pt.z = 0.0;
	brd.points.push_back(line_pt);
	line_pt.x = -border_width/2;
	line_pt.y = -border_height/2;
	line_pt.z = 0.0;
	brd.points.push_back(line_pt);
	pub_borders.publish(brd);
	
	// store robot xy position in a rigid transformation
	Vector2D robot_pos(dd.getX(), dd.getY());
	Transform2D t(robot_pos, dd.getTheta());

	int count = 0;
	int laser_count = 0;
	while(n.ok())
	{	
		ros::spinOnce();
		current_time = ros::Time::now(); 
		double dt = (current_time - last_time).toSec();
		// update pose of the robot given wheel velocities
		dd.updatePose(dd.getLWheelPhi()+dt*wv.l_vel, dd.getRWheelPhi()+dt*wv.r_vel, slip_unif(get_random()), slip_unif(get_random()));
		// check for collisions
		for(std::size_t i = 0; i < x_tubes.size(); ++i)
		{	
			// absolute position (wrt world frame) of the obstacle
			Vector2D p_abs(x_tubes[i], y_tubes[i]);
			// absolute position (wrt world frame) of the robot
			Vector2D robot_pos(dd.getX(), dd.getY());
			// position vector difference
			Vector2D p_diff = (p_abs - robot_pos);
			// magnitude of current distance
			double d_current = magnitude(p_diff);
			
			if (d_current < d_crit)
			{
				// normalize position vector difference
				p_diff.normalize();
				// change in robot position commanded			
				Vector2D dp = -(d_crit - d_current)*p_diff;
				// edit robot pos
				robot_pos.x += dp.x;
				robot_pos.y += dp.y;
				// construct the new robot pose (heading angle does not change)
				RobotPose rp{dd.getTheta(), robot_pos.x, robot_pos.y};
				// change the robot pose
				dd.setPose(rp);
			}
		}
		js.header.stamp = current_time;
		js.position = {dd.getLWheelPhi(), dd.getRWheelPhi()};
		js.velocity = {wv.l_vel, wv.r_vel};
		pub.publish(js);
		
		// Publish frame transformation given new pose of the robot
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
		
		//Publish path given new pose of the robot
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
		
		if (count == 10)
		{
			// store robot xy position in a rigid transformation
			Vector2D robot_pos(dd.getX(), dd.getY());
			Transform2D t(robot_pos, dd.getTheta());
			// find inverse of transformation
			Transform2D tinv = t.inv();
			// contains position of the markers relative to the robot frame
			visualization_msgs::MarkerArray relative_marker_arr;
			for(std::size_t i = 0; i < x_tubes.size(); ++i)
			{
				visualization_msgs::Marker m;  // define new marker to add to the array
				m.header.stamp = current_time; 
				m.header.frame_id = "turtle";  // position of marker is relative to the robot frame
				m.ns = "relative";
				m.id = i+1;  // unique id in namespace "relative"
				m.type = visualization_msgs::Marker::CYLINDER;
				
				Vector2D p_abs(x_tubes[i], y_tubes[i]);  // store position of the tube with respect to the world frame in a vector
				std::vector<double> noise_vec = mv_xy.draw();
				Vector2D p_noise((double)noise_vec[0], (double)noise_vec[1]);
				
				Vector2D p_rel = tinv(p_abs) + p_noise;  // find position of tubes in the robot frame and add noise
				double mag = magnitude(p_rel);  // calculate sensed robot-tube distance
				
				// Check if distance to obstacle is above sensing range
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
			pub_sensor.publish(relative_marker_arr);  // publish relative obstacle pose markers
			count = 0;
		}
		else
		{
			++count;
		}
		
		if (laser_count == 20)  // publish lidar measurements at 5 Hz
		{
			sensor_msgs::LaserScan simulated_laser;
			simulated_laser.header.stamp = current_time;
			simulated_laser.header.frame_id = "turtle";
			simulated_laser.angle_min = 0;
			simulated_laser.angle_increment = deg2rad(lidar_dtheta);
			simulated_laser.range_min = lidar_min_range;
			simulated_laser.range_max = lidar_max_range;
			simulated_laser.angle_max = deg2rad(lidar_dtheta)*(lidar_n_samples-1);
			for (int i=0; i<lidar_n_samples; ++i)
			{
				double relative_scan_angle = i*simulated_laser.angle_increment; // current lidar angle in robot frame
				double absolute_scan_angle = normalize_angle(relative_scan_angle + dd.getTheta());  // current lidar angle in world frame
				Vector2D p1(dd.getX(), dd.getY());  // robot xy coords wrt world frame
				Vector2D p2;  // point on the border coords wrt world frame
				double alpha = atan((0.5*border_height-dd.getY())/(0.5*border_width-dd.getX()));
				double beta = atan((0.5*border_height+dd.getY())/(0.5*border_width-dd.getX()));
				double gamma = atan((0.5*border_height-dd.getY())/(0.5*border_width+dd.getX()));
				double delta = atan((0.5*border_height+dd.getY())/(0.5*border_width+dd.getX()));
				if ((absolute_scan_angle >= -beta) && (absolute_scan_angle <= alpha))  // west wall
				{
					p2.x = border_width/2;
					p2.y = p1.y + tan(absolute_scan_angle)*(p2.x - p1.x);
				}
				else if ((absolute_scan_angle > alpha) && (absolute_scan_angle < PI-gamma)) // north wall
				{
					p2.y = border_height/2;
					p2.x = p1.x + (p2.y - p1.y)/tan(absolute_scan_angle);					
				}
				else if ((absolute_scan_angle >= PI-gamma) || (absolute_scan_angle <= -PI+delta)) // east wall
				{
					p2.x = -border_width/2;
					p2.y = p1.y + tan(absolute_scan_angle)*(p2.x - p1.x);
				}
				else if ((absolute_scan_angle > -PI+delta) && (absolute_scan_angle < -beta)) // south wall
				{
					p2.y = -border_height/2;
					p2.x = p1.x + (p2.y - p1.y)/tan(absolute_scan_angle);
				}
				double range = magnitude(p1-p2);
				Intersection it;
				for(std::size_t i = 0; i < x_tubes.size(); ++i)
				{
					Vector2D p_circle(x_tubes[i], y_tubes[i]);
					Intersection it_temp = compute_intersection(p1, p2, p_circle, tube_radius);
					if (it_temp.is_intersection)
					{
						if (!it.is_intersection)
						{
							Vector2D p1_i_new(p1.x-it_temp.x, p1.y-it_temp.y);
							range = magnitude(p1_i_new);
							it = it_temp;
						}
						else
						{
							Vector2D p1_i_new(p1.x-it_temp.x, p1.y-it_temp.y);
							if (magnitude(p1_i_new) < range)  // lidar only sees the closest object
							{
								it = it_temp;
								range = magnitude(p1_i_new);
							}
						}
					}
				}
				int num_resolutions = range/lidar_resolution;  // convert simulated data according to lidar resolution
				range = num_resolutions * lidar_resolution;
				simulated_laser.ranges.push_back(range);
			}
			pub_laser.publish(simulated_laser);
			laser_count = 0;
		}
		else
		{
			++laser_count;
		}
		last_time = current_time;
		r.sleep();
	}
	return 0;
}
