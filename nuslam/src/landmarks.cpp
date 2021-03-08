/// \file
/// \brief node to detect landmarks and publish their relative locations
///
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
	
	double thresh = 0.3;  // distance threshold for clustering
	int min_n = 3;  // minimum number of elements in a cluster
	float min_mean = deg2rad(90);
	float max_mean = deg2rad(135);
	float max_std = 0.15;
	
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
			clusters = cluster_ranges(scan.ranges, thresh, min_n);
			for (std::size_t i=0; i<clusters.size(); ++i)
			{
				if (is_circle(scan.ranges, clusters[i], scan.angle_increment, min_mean, max_mean, max_std))
				{
					std::vector<Vector2D> pts = range2xy(scan.ranges, clusters[i], scan.angle_increment);
					Circle c = fit_circle(pts);
					circles.push_back(c);
				}
			}
			for (std::size_t i=0; i<circles.size(); ++i)
			{
				visualization_msgs::Marker m;
				m.header.stamp = current_time;
				m.header.frame_id = "map";  // relative to the map frame
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
