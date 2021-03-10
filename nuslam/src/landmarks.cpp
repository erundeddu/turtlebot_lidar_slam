/// \file
/// \brief node to detect landmarks and publish their relative locations
///
/// PARAMETERS:
/// 	min_cluster_n (int): minimum number of elements in a cluster
/// 	min_angle_mean (double): minimum angle mean for circle (degrees)
///		max_angle_mean (double): maximum angle mean for circle (degrees)
///		max_angle_std (double): maximum angle standard deviation for circle (radians)
///		cluster_d_thresh (double): measurement distance threshold for clustering (m)
///		min_circle_radius (double): minimum radius for circle (m)
///		max_circle_radius (double): maximum radius for circle (m)
/// PUBLISHES:
///     circles_detected (visualization_msgs/MarkerArray): circles detected from laser data
/// SUBSCRIBES:
///     scan (sensor_msgs/LaserScan): lidar data

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "nuslam/circle_learning.hpp"
#include "rigid2d/rigid2d.hpp"
#include <vector>

static bool is_received = false;
static sensor_msgs::LaserScan scan;

/// \brief Receives lidar data
/// \param msg - a pointer to the sensor_msgs/LaserScan message containing lidar data
void callback(const sensor_msgs::LaserScan::ConstPtr & msg)
{
	scan = *msg;
	is_received = true;
}

int main(int argc, char** argv)
{
	using namespace nuslam;
	using namespace rigid2d;
	
	ros::init(argc, argv, "landmarks");
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Subscriber sub = n.subscribe("scan", 1000, callback);
	ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("circles_detected", 1000);
	
	double cluster_d_thresh = 0.3;  // distance threshold for clustering
	int min_cluster_n = 3;  // minimum number of elements in a cluster
	double min_angle_mean_deg = 90;
	double max_angle_mean_deg = 135;
	double max_angle_std = 0.15;
	double min_circle_radius = 0.03;
	double max_circle_radius = 0.14;
	
	
	n.getParam("cluster_d_thresh", cluster_d_thresh);
	n.getParam("min_cluster_n", min_cluster_n);
	n.getParam("min_angle_mean", min_angle_mean_deg);
	n.getParam("max_angle_mean", max_angle_mean_deg);
	n.getParam("max_angle_std", max_angle_std);
	n.getParam("min_circle_radius", min_circle_radius);
	n.getParam("max_circle_radius", max_circle_radius);
	
	double min_angle_mean = deg2rad(min_angle_mean_deg);
	double max_angle_mean = deg2rad(max_angle_mean_deg);
	
	
	std::vector<std::vector<int>> clusters;
	auto current_time = ros::Time::now();
	
	while(n.ok())
	{	
		ros::spinOnce();
		current_time = ros::Time::now(); 
		std::vector<Circle> circles;
		visualization_msgs::MarkerArray circle_markers;
		if (is_received)
		{
			clusters = cluster_ranges(scan.ranges, cluster_d_thresh, min_cluster_n);
			for (std::size_t i=0; i<clusters.size(); ++i)
			{
				if (is_circle(scan.ranges, clusters[i], scan.angle_increment, min_angle_mean, max_angle_mean, max_angle_std))
				{
					std::vector<Vector2D> pts = range2xy(scan.ranges, clusters[i], scan.angle_increment);
					Circle c = fit_circle(pts);
					if ((c.r >= min_circle_radius) && (c.r <= max_circle_radius))
					{
						circles.push_back(c);
					}
				}
			}
			for (std::size_t i=0; i<circles.size(); ++i)
			{
				visualization_msgs::Marker m;
				m.header.stamp = current_time;
				m.header.frame_id = "turtle";  // relative to the robot frame
				m.ns = "circles";
				m.id = i+1;  // unique id under namespace
				m.type = visualization_msgs::Marker::CYLINDER;
				m.action = visualization_msgs::Marker::ADD;
				// assign x and y positions of the marker to be the center of the circle fitted
				m.pose.position.x = circles[i].x; 
				m.pose.position.y = circles[i].y;
				m.pose.position.z = 0;
				m.pose.orientation.x = 0;
				m.pose.orientation.y = 0;
				m.pose.orientation.z = 0;
				m.pose.orientation.w = 1;
				// scale rviz visualization with fitted radius
				m.scale.x = circles[i].r;
				m.scale.y = circles[i].r;
				m.scale.z = 0.2;
				// yellow marker, not transparent
				m.color.r = 0.0;
				m.color.b = 1.0;
				m.color.g = 1.0;
				m.color.a = 1.0;
				circle_markers.markers.push_back(m);  // add marker to the array
			}
			pub.publish(circle_markers);
		}
		r.sleep();
	}
	
	return 0;
}
