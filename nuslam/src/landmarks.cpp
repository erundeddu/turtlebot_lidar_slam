/// \file
/// \brief node to detect landmarks and publish their relative locations
///
/// PUBLISHES:
///     circles_detected (visualization_msgs/MarkerArray): circles detected from laser data
/// SUBSCRIBES:
///     scan (sensor_msgs/LaserScan): lidar data

#include <sensor_msgs/LaserScan.h>
#include "nuslam/circle_learning.hpp"

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
	ros::init(argc, argv, "landmarks");
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Subscriber sub = n.subscribe("scan", 1000, callback);
	ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("circles_detected", 1000);
	
	while(n.ok())
	{	
		if (is_received)
		{
			//do this TODO
		}
		r.sleep();
	}
	
	return 0;
}
