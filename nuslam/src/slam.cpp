/// \file
/// \brief performs SLAM for a robot given landmark makers and publishes rviz compatible paths for odometry-only pose estimate and SLAM pose estimate
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
///		unknown_assoc (int): if true (1), subscribe to circles_detected and perform unknwon data association. if false (0) subscribe to fake_sensor and use landmark id's
///		mahdst_thresh (double): mahalanobis distance threshold to define new landmark
/// PUBLISHES:
///     odom (nav_msgs/Odometry): robot pose in the odom_frame_id frame, robot body velocity in body_frame_id
///		odom_path (nav_msgs/Path): robot path according to odometry only
///		slam_path (nav_msgs/Path): robot path according to SLAM
///		slam_map (visualization_msgs/MarkerArray): landmark state from SLAM
/// SUBSCRIBES:
///     joint_states (sensor_msgs/JointState): angles and angular velocities of left and right robot wheels
///	    fake_sensor (visualization_msgs/MarkerArray): landmarks seen by the robot
///		circles_detected (visualization_msgs/MarkerArray): landmarks extracted from laser data
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
static nav_msgs::Odometry odom;
static geometry_msgs::TransformStamped odom2body;
static geometry_msgs::TransformStamped world2map;
static geometry_msgs::TransformStamped map2odom;
static bool started(false);
static std::queue<nuslam::Landmark> qe;
static arma::Col<double> M;
static nav_msgs::Path odom_path;
static nav_msgs::Path slam_path;
static std::vector<int> is_init;
int n_lm = 10;  // maximum number of landmarks to track
static geometry_msgs::PoseStamped pos;

static rigid2d::Transform2D t_mb;  // map to base_footprint transform
static rigid2d::Transform2D t_mo;  // map to odom transform
static rigid2d::Transform2D t_ob;  // odom to base_footprint transform

// matrices for SLAM
static arma::Mat<double> S;
static arma::Mat<double> R;
static arma::Mat<double> Q_bar;

static int unknown_assoc = 0;
static int lm_seen = 0;
static double mahdst_thresh = 1.0;


/// \brief Updates internal odometry state, publishes a ROS odometry message, broadcast the transform between odometry and body frame on tf
/// \param msg - a pointer to the sensor_msg/JointState message with angles and angular velocities of the robot wheels
void callback(const sensor_msgs::JointState::ConstPtr & msg)
{
	using namespace rigid2d;
	using namespace nuslam;
	
	static ros::NodeHandle nh;
	static ros::Time current_time;
	static ros::Time last_time;
	current_time = ros::Time::now();
	
	// update position in robot
	double l_phi_wheel_new = msg -> position[0];
	double r_phi_wheel_new = msg -> position[1];

	// update odometry message
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
	
	arma::Mat<double> A = compute_A_mat(dd, t_mb, l_phi_wheel_new, r_phi_wheel_new, n_lm);
	dd.updatePose(l_phi_wheel_new, r_phi_wheel_new);  // update robot odometry
	
	Vector2D v_bb(dd.getX(), dd.getY());  
	Transform2D t_bb(v_bb, dd.getTheta());  
	t_ob = t_bb;  // get new odometry state
	t_mb = t_mo*t_ob;  // recalculate map to base_footprint transform
	
	// construct message for odom path
	pos.pose.position.x = t_ob.getX();
	pos.pose.position.y = t_ob.getY();
	pos.pose.position.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, t_ob.getTheta());
	pos.pose.orientation.x = q.x();
	pos.pose.orientation.y = q.y();
	pos.pose.orientation.z = q.z();
	pos.pose.orientation.w = q.w();
	odom_path.poses.push_back(pos);
		
	S = A*S*arma::trans(A) + Q_bar;  // propagate uncertainty
	
	while (!qe.empty())  // if markers message are available, process them
	{
		Landmark lm = qe.front();  // get landmark
		qe.pop();  // delete landmark from queue
		
		//FIXME do data association here (start)
		if (unknown_assoc)
		{
			arma::Mat<double> Mcopy(M);  // make a copy of the landmarks matrix
			Landmark lm_temp(lm.r, lm.phi, lm_seen+1);  // process incoming landmark as new landmark
			initialize_landmark(t_mb, lm_temp, Mcopy);  // temporarily initialize_landmark
			arma::Col<double> z = {lm_temp.r, lm_temp.phi};
			std::vector<double> mahdst(lm_seen+1);  // vector of mahalanobis distance
			for (int j=0; j<lm_seen+1; ++j)
			{
				double dx = Mcopy(2*j,0) - t_mb.getX();
				double dy = Mcopy(2*j+1,0) - t_mb.getY();
				
				arma::Mat<double> H = compute_Hj_mat(dx, dy, n_lm, j+1);  // compute Hk, the linearized measurement model
				arma::Mat<double> psi = H*S*H.t() + R;  // compute the covariance
				arma::Col<double> zh = compute_meas(t_mb, Mcopy, j+1);
				arma::Col<double> delta_z = {z(0,0)-zh(0,0), normalize_angular_difference(z(1,0), zh(1,0))};
				arma::Mat<double> dk = delta_z.t() * arma::inv(psi) * delta_z;
				mahdst[j] = dk(0,0);
				
			}
			mahdst[lm_seen] = mahdst_thresh;
			double dk_min = 100000;
			int l=0;
			for (int j=0; j<lm_seen+1; ++j)
			{
				double dk_current = mahdst[j];
				if (dk_current < dk_min)
				{
					dk_min = dk_current;
					l=j;
				}
			}
			lm.id = l+1;
			if (l < n_lm-1)
			{
				if (l == lm_seen)
				{
					++lm_seen;
				}
			}
		}
		//FIXME do that association here (end)
			
		if (!is_init[lm.id-1])  // landmark id's start from 1
		{
			initialize_landmark(t_mb, lm, M);
			is_init[lm.id-1] = 1;
		}
		// compute the theoretical measurement
		arma::Col<double> zh = compute_meas(t_mb, M, lm.id);
		// compute the kalman gain
		double dx = M(2*(lm.id-1),0) - t_mb.getX();
		double dy = M(2*(lm.id-1)+1,0) - t_mb.getY();
		arma::Mat<double> H = compute_Hj_mat(dx, dy, n_lm, lm.id);
		arma::Mat<double> K = S*H.t()*arma::inv(H*S*H.t()+R);
		// compute the posterior state update
		arma::Col<double> z = {lm.r, lm.phi};
		arma::Col<double> q = {t_mb.getTheta(), t_mb.getX(), t_mb.getY()};
		arma::Col<double> state = arma::join_cols(q, M);
		arma::Col<double> delta_z = {z(0,0)-zh(0,0), normalize_angular_difference(z(1,0), zh(1,0))};
		state += K*delta_z;
		// update internal transformations and states
		Vector2D v_mb_goal(state(1,0), state(2,0));
		Transform2D t_mb_goal(v_mb_goal, state(0,0));
		t_mo = t_mb_goal*t_ob.inv();
		t_mb = t_mb_goal;
		M = state.rows(3,M.n_rows+2);
		// compute the posterior covariance
		S = (arma::eye(size(S)) - K*H)*S;
	}

	// construct message for SLAM path
	pos.header.stamp = current_time;
	pos.pose.position.x = t_mb.getX();
	pos.pose.position.y = t_mb.getY();
	pos.pose.position.z = 0.0;
	q.setRPY(0, 0, t_mb.getTheta());
	pos.pose.orientation.x = q.x();
	pos.pose.orientation.y = q.y();
	pos.pose.orientation.z = q.z();
	pos.pose.orientation.w = q.w();
	slam_path.poses.push_back(pos);
	
	// construct odometry message	
	odom.pose.pose.position.x = t_ob.getX();
	odom.pose.pose.position.y = t_ob.getY();
	odom.pose.pose.position.z = 0.0;
	q.setRPY(0, 0, t_ob.getTheta());
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();
	
	// construct odom to base_footprint transform
	odom2body.transform.translation.x = dd.getX();
	odom2body.transform.translation.y = dd.getY();
	odom2body.transform.translation.z = 0.0;
	odom2body.transform.rotation.x = q.x();
	odom2body.transform.rotation.y = q.y();
	odom2body.transform.rotation.z = q.z();
	odom2body.transform.rotation.w = q.w();
	
	// construct map to odom transform
	map2odom.transform.translation.x = t_mo.getX();
	map2odom.transform.translation.y = t_mo.getY();
	map2odom.transform.translation.z = 0;
	q.setRPY(0, 0, t_mo.getTheta());
	map2odom.transform.rotation.x = q.x();
	map2odom.transform.rotation.y = q.y();
	map2odom.transform.rotation.z = q.z();
	map2odom.transform.rotation.w = q.w();
		
	last_time = current_time;
	started = true;
}


/// \brief receives information about markers and adds them to a queue to be processed by the slam loop
/// \param msg - a pointer to the visualization_msgs/MarkerArray message containing all simulated markers seen by the robot
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
			qe.push(lm);
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
	using namespace nuslam;
	
	ros::init(argc, argv, "slam");
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Subscriber sub = n.subscribe("joint_states", 1000, callback);	
	ros::Subscriber sub_mark;
	ros::Publisher pub_odom = n.advertise<nav_msgs::Path>("odom_path", 1000);
	ros::Publisher pub_slam = n.advertise<nav_msgs::Path>("slam_path", 1000);
	ros::Publisher pub_mark = n.advertise<visualization_msgs::MarkerArray>("slam_map", 1000);
	ros::Publisher pub_nav = n.advertise<nav_msgs::Odometry>("odom", 1000);
	tf2_ros::TransformBroadcaster broadcaster;
	ros::ServiceServer srv = n.advertiseService("set_pose", set_pose_method);
	
	double wheel_base;
	double wheel_radius;
	std::string odom_frame_id;
	std::string body_frame_id;
	std::string left_wheel_joint;
	std::string right_wheel_joint;
	std::vector<double> q_cov; 
	std::vector<double> r_cov;
	
	n.getParam("wheel_base", wheel_base);
	n.getParam("wheel_radius", wheel_radius);
	n.getParam("odom_frame_id", odom_frame_id);
	n.getParam("body_frame_id", body_frame_id);
	n.getParam("left_wheel_joint", left_wheel_joint);
	n.getParam("right_wheel_joint", right_wheel_joint);
	n.getParam("q_cov", q_cov);
	n.getParam("r_cov", r_cov);
	n.getParam("unknown_assoc", unknown_assoc);
	n.getParam("mahdst_thresh", mahdst_thresh);
	
	if (unknown_assoc)
	{
		sub_mark = n.subscribe("circles_detected", 1000, markers_callback);
	}
	else
	{
		sub_mark = n.subscribe("fake_sensor", 1000, markers_callback);
	}
	
	odom.header.frame_id = odom_frame_id;
	odom.child_frame_id = body_frame_id;
	odom2body.header.frame_id = odom_frame_id;
	odom2body.child_frame_id = body_frame_id;
	
	world2map.header.frame_id = "world";
	world2map.child_frame_id = "map";
	
	map2odom.header.frame_id = "map";
	map2odom.child_frame_id = odom_frame_id;
	
	slam_path.header.frame_id = "map";
	odom_path.header.frame_id = "map";
	
	dd.setPhysicalParams(wheel_base, wheel_radius);
	
	M = arma::zeros(2*n_lm,1);  // initialize map vector
	
	for (int i=0; i < n_lm; ++i) // initialize vector to check for landmark initialization
	{
		is_init.push_back(0);
	}
										
	S = initialize_S(n_lm);
	R = initialize_R(r_cov);
	Q_bar = initialize_Qbar(q_cov, n_lm);	
	
	while(n.ok())
	{
		ros::spinOnce(); 
		auto current_time = ros::Time::now();
		visualization_msgs::MarkerArray slam_markers;
		for (int i=0; i < n_lm; ++i)
		{
			if (is_init[i])
			{
				visualization_msgs::Marker m;  // initialize marker to add to the array
				m.header.stamp = current_time;
				m.header.frame_id = "map";
				m.ns = "slam";
				m.id = i+1;  // unique id under namespace "real"
				m.type = visualization_msgs::Marker::CYLINDER;
				m.action = visualization_msgs::Marker::ADD;
				// assign x and y positions of the marker as specified by the ros parameter
				m.pose.position.x = M(2*i,0);
				m.pose.position.y = M(2*i+1,0);
				m.pose.position.z = 0;
				m.pose.orientation.x = 0;
				m.pose.orientation.y = 0;
				m.pose.orientation.z = 0;
				m.pose.orientation.w = 1;
				// scale rviz visualization with tube_radius
				m.scale.x = 0.07;
				m.scale.y = 0.07;
				m.scale.z = 0.2;
				m.color.r = 0.0;
				m.color.b = 0.1;
				m.color.g = 0.0;
				m.color.a = 1.0;
				slam_markers.markers.push_back(m);  // add marker to the array
			}
		}
		
		world2map.transform.translation.x = 0;
		world2map.transform.translation.y = 0;
		world2map.transform.translation.z = 0;
		world2map.transform.rotation.x = 0;
		world2map.transform.rotation.y = 0;
		world2map.transform.rotation.z = 0;
		world2map.transform.rotation.w = 1;
		
		odom_path.header.stamp = current_time;
		pos.header.stamp = current_time;
		odom.header.stamp = current_time;
		slam_path.header.stamp = current_time;
		world2map.header.stamp = current_time;
		map2odom.header.stamp = current_time;
		odom2body.header.stamp = current_time;
		
		pub_mark.publish(slam_markers);
		pub_odom.publish(odom_path);
		pub_slam.publish(slam_path);
		pub_nav.publish(odom);
		
		broadcaster.sendTransform(world2map);
		broadcaster.sendTransform(odom2body);
		broadcaster.sendTransform(map2odom);
		
		r.sleep();
	}
	
	return 0;
}
