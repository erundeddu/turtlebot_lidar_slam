/// \file
/// \brief Publishes odometry messages in a standard ROS way
///
/// PARAMETERS:
///		wheel_base (double): the distance between the robot wheels
///		wheel_radius (double): the radius of the wheels
///		left_wheel_joint (string): the name of the left wheel joint
///		right_wheel_joint (string): the name of the right wheel joint
///		odom_frame_id (string): the name of the odometry tf frame
///		body_frame_id (string): the name of the body tf frame
///		q_cov (std::vector<double>): covariance matrix for odometry process noise
///		r_cov (std::vector<double>): covariance matrix for sensor noise
/// PUBLISHES:
///     odom (nav_msgs/Odometry): robot pose in the odom_frame_id frame, robot body velocity in body_frame_id
///		odom_path (nav_msgs/Path): robot path according to odometry only
///		slam_path (nav_msgs/Path): robot path according to SLAM
/// SUBSCRIBES:
///     joint_states (sensor_msgs/JointState): angles and angular velocities of left and right robot wheels
///	    fake_sensor (visualization_msgs/MarkerArray): landmarks seen by the robot
/// SERVICES:
///		set_pose (rigid2d/set_pose): provides a new pose to change where the robot think it is

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <random>
#include <vector>
#include <queue>
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/set_pose.h"
#include "nuslam/helper.hpp"

static rigid2d::DiffDrive dd;
static rigid2d::DiffDrive dd_odom;
static nav_msgs::Odometry odom;
static geometry_msgs::TransformStamped odom_trans;
static bool started(false);
static std::vector<double> q_cov; 
static std::vector<double> r_cov;
static std::queue<nuslam::Landmark> qe;
static arma::Col<double> M;
int n_lm = 10;  // maximum number of landmarks to track


/// \brief Updates internal odometry state, publishes a ROS odometry message, broadcast the transform between odometry and body frame on tf
/// \param msg - a pointer to the sensor_msg/JointState message with angles and angular velocities of the robot wheels
void callback(const sensor_msgs::JointState::ConstPtr & msg)
{
	using namespace rigid2d;
	using namespace nuslam;
	
	static ros::NodeHandle nh;
	static tf2_ros::TransformBroadcaster broadcaster;
	static ros::Publisher pub_odom = nh.advertise<nav_msgs::Path>("odom_path", 1000);
	static ros::Publisher pub_slam = nh.advertise<nav_msgs::Path>("slam_path", 1000);
	static ros::Time current_time;
	static ros::Time last_time;
	current_time = ros::Time::now();
	
	// update position in robot
	double l_phi_wheel_new = msg -> position[0];
	double r_phi_wheel_new = msg -> position[1];

	if (started)
	{
		double dt = (current_time - last_time).toSec();
		Twist2D tw = dd.getBodyTwist(l_phi_wheel_new, r_phi_wheel_new, dt);
		odom.twist.twist.linear.x = tw.getVx();
		odom.twist.twist.linear.y = tw.getVy();
		odom.twist.twist.angular.z = tw.getW();
	}
	else
	{
		odom.twist.twist.linear.x = 0;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = 0;
	}
	
	//FIXME document and order these initializations
	arma::Mat<double> A = compute_A_mat(dd, l_phi_wheel_new, r_phi_wheel_new, n_lm);
	dd.updatePose(l_phi_wheel_new, r_phi_wheel_new);  //update estimate of the model (odometry only)
	dd_odom.updatePose(l_phi_wheel_new, r_phi_wheel_new);  //update estimate of the model (odometry only)
	
	// publish odom only path
	static nav_msgs::Path odom_path;
	odom_path.header.frame_id = "world";  /// FIXME which frame to use?
	odom_path.header.stamp = current_time;
	static geometry_msgs::PoseStamped pos;
	pos.header.stamp = current_time;
	pos.pose.position.x = dd_odom.getX();
	pos.pose.position.y = dd_odom.getY();
	pos.pose.position.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, dd_odom.getTheta());
	pos.pose.orientation.x = q.x();
	pos.pose.orientation.y = q.y();
	pos.pose.orientation.z = q.z();
	pos.pose.orientation.w = q.w();
	odom_path.poses.push_back(pos);
	pub_odom.publish(odom_path);
	
	
	static arma::Mat<double> Q = {	{q_cov[0], q_cov[3], q_cov[4]},
									{q_cov[3], q_cov[1], q_cov[5]},
									{q_cov[4], q_cov[5], q_cov[2]}};
										
	static arma::Mat<double> S = {arma::join_cols(arma::join_rows(arma::zeros(3,3), arma::zeros(3,2*n_lm)), arma::join_rows(arma::zeros(2*n_lm,3), 10000*arma::eye(2*n_lm,2*n_lm)))};  // initialize sigma matrix (sigma_0)
	
	static arma::Mat<double> R = {	{r_cov[0], r_cov[2]},
									{r_cov[2], r_cov[1]}};
								
	static std::vector<int> is_init;
	for (int i=0; i < n_lm; ++i)
	{
		is_init.push_back(0);
	}
		
	static arma::Mat<double> Q_bar = arma::join_cols(arma::join_rows(Q, arma::zeros(3,2*n_lm)), arma::join_rows(arma::zeros(2*n_lm,3), arma::zeros(2*n_lm,2*n_lm)));
	
	
	S = A*S*arma::trans(A) + Q_bar;  // propagate uncertainty
	
	while (!qe.empty())
	{
		Landmark lm = qe.front();
		qe.pop();
		if (!is_init[lm.id-1])  // landmark id's start from 1
		{
			initialize_landmark(dd, lm, M);
			is_init[lm.id-1] = 1;
		}
		//TODO move to helper if necessary
		// compute the theoretical measurement
		arma::Col<double> zh = compute_meas(dd, M, lm.id);
		// compute the kalman gain
		double dx = M(2*(lm.id-1),0) - dd.getX();
		double dy = M(2*(lm.id-1)+1,0) - dd.getY();
		arma::Mat<double> H = compute_Hj_mat(dx, dy, n_lm, lm.id);
		arma::Mat<double> K = S*H.t()*arma::inv(H*S*H.t()+R);
		// compute the posterior state update
		arma::Col<double> z = {lm.r, lm.phi};
		arma::Col<double> q = {dd.getTheta(), dd.getX(), dd.getY()};
		arma::Col<double> state = arma::join_cols(q, M);
		state += K*(z-zh);
		// update internal variables
		RobotPose rp_new{state(0,0), state(1,0), state(2,0)};
		dd.setPose(rp_new);
		M = state.rows(3,M.n_rows+2);
		// compute the posterior covariance
		S = (arma::eye(size(S)) - K*H)*S;
	}

	
	// publish SLAM path
	static nav_msgs::Path slam_path;
	slam_path.header.frame_id = "world";  /// FIXME which frame to use?
	slam_path.header.stamp = current_time;
	pos.header.stamp = current_time;
	pos.pose.position.x = dd.getX();
	pos.pose.position.y = dd.getY();
	pos.pose.position.z = 0.0;
	q.setRPY(0, 0, dd.getTheta());
	pos.pose.orientation.x = q.x();
	pos.pose.orientation.y = q.y();
	pos.pose.orientation.z = q.z();
	pos.pose.orientation.w = q.w();
	slam_path.poses.push_back(pos);
	pub_slam.publish(slam_path);
	

	// start referencing http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom here (Access: 2/1/2021)	
	odom.header.stamp = current_time;
	odom.pose.pose.position.x = dd.getX();
	odom.pose.pose.position.y = dd.getY();
	odom.pose.pose.position.z = 0.0;

	q.setRPY(0, 0, dd.getTheta());
	
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();
	
	static ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
	pub.publish(odom);
	
	odom_trans.header.stamp = current_time;
	odom_trans.transform.translation.x = dd.getX();
	odom_trans.transform.translation.y = dd.getY();
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation.x = q.x();
	odom_trans.transform.rotation.y = q.y();
	odom_trans.transform.rotation.z = q.z();
	odom_trans.transform.rotation.w = q.w();
	broadcaster.sendTransform(odom_trans);
		
	last_time = current_time;
	started = true;
	// end referencing http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom here (Access: 2/1/2021)
}


//TODO comment
void markers_callback(const visualization_msgs::MarkerArray::ConstPtr & msg)
{
	using namespace nuslam;
	using namespace rigid2d;
	int num_m = msg -> markers.size();
		for (int i = 0; i < num_m; ++i)
		{
			if (msg -> markers[i].action == 0)
			{
				Vector2D v(msg -> markers[i].pose.position.x, msg -> markers[i].pose.position.y);
				Landmark lm(magnitude(v), angle(v), msg -> markers[i].id);
				//if ((int)qe.size() < n_lm) 
				//{	
				qe.push(lm);
				//}
			}
		}
}


/// \brief following a set_pose service, reset location of odometry to match requested configuration
/// \param req - service request package of type rigid2d::set_pose::Request (containing robot pose)
/// \param res - service response package of type rigid2d::set_pose::Response (empty)
/// \return true if services were successfully called, else false
bool set_pose_method(rigid2d::set_pose::Request & req, rigid2d::set_pose::Response &)
{
	using namespace rigid2d;
	
	RobotPose q;
	q.theta = req.theta;
	q.x = req.x;
	q.y = req.y;
	dd.setPose(q);
	return true;
}


int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "slam");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joint_states", 1000, callback);
	ros::Subscriber sub_mark = n.subscribe("fake_sensor", 1000, markers_callback);
	
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
	n.getParam("q_cov", q_cov);
	n.getParam("r_cov", r_cov);
	
	odom.header.frame_id = odom_frame_id;
	odom.child_frame_id = body_frame_id;
	odom_trans.header.frame_id = odom_frame_id;
	odom_trans.child_frame_id = body_frame_id;
	
	dd.setPhysicalParams(wheel_base, wheel_radius);
	dd_odom.setPhysicalParams(wheel_base, wheel_radius);
	ros::ServiceServer srv = n.advertiseService("set_pose", set_pose_method);
	ros::Rate r(100);
	M = arma::zeros(2*n_lm,1);
	
	while(n.ok())
	{
		ros::spinOnce(); 
		r.sleep();
	}
	
	return 0;
}