#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "trect/start.h"
#include "std_srvs/Empty.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_msgs/String.h"


static double x;
static double y;
static double theta;
static double x_rect;
static double y_rect;
static double width;
static double height;
static int stage = 0;

void callback(const turtlesim::Pose::ConstPtr& msg)
{
	x = msg->x;
	y = msg->y;
	theta = msg->theta;
}

bool start_method(trect::start::Request & req, trect::start::Response & res)
{
	ros::NodeHandle n;  //TODO: should a new NodeHandle be used?ros
	x_rect = req.x_rect;
	y_rect = req.y_rect;
	width = req.width;
	height = req.height;
	
	ros::ServiceClient clearClient = n.serviceClient<std_srvs::Empty>("clear");  // source: https://www.cse.sc.edu/~jokane/agitr/agitr-small.pdf
	std_srvs::Empty::Request req_clear;
	std_srvs::Empty::Response res_clear;
	
	if (!clearClient.call(req_clear, res_clear))
	{
		ROS_ERROR("Failed to call /clear");
	}
	
	ros::ServiceClient setpenClient = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
	turtlesim::SetPen::Request req_setpen;
	turtlesim::SetPen::Response res_setpen;
	req_setpen.r = 255;
	req_setpen.g = 255;
	req_setpen.b = 255;
	req_setpen.width = 2;
	req_setpen.off = 1;
	
	if (!setpenClient.call(req_setpen, res_setpen))
	{
		ROS_ERROR("Failed to call /set_pen");
	}
	
	ros::ServiceClient teleportabsClient = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
	turtlesim::TeleportAbsolute::Request req_telepabs;
	turtlesim::TeleportAbsolute::Response res_telepabs;
	req_telepabs.x = x_rect;
	req_telepabs.y = y_rect;
	req_telepabs.theta = 0.0;
	
	if (!teleportabsClient.call(req_telepabs, res_telepabs))
	{
		ROS_ERROR("Failed to call /teleport_absolute");
	}
	
	req_setpen.off = 0;
	if (!setpenClient.call(req_setpen, res_setpen))
	{
		ROS_ERROR("Failed to call /set_pen");
	}
	
	req_telepabs.x = x_rect + width;
	if (!teleportabsClient.call(req_telepabs, res_telepabs))
	{
		ROS_ERROR("Failed to call /teleport_absolute");
	}
	
	req_telepabs.y = y_rect + height;
	if (!teleportabsClient.call(req_telepabs, res_telepabs))
	{
		ROS_ERROR("Failed to call /teleport_absolute");
	}
	
	req_telepabs.x = x_rect;
	if (!teleportabsClient.call(req_telepabs, res_telepabs))
	{
		ROS_ERROR("Failed to call /teleport_absolute");
	}
	
	req_telepabs.y = y_rect;
	if(!teleportabsClient.call(req_telepabs, res_telepabs))
	
	req_setpen.r = 0;
	req_setpen.g = 0;
	req_setpen.b = 0;
	if(!setpenClient.call(req_setpen, res_setpen))
	{
		ROS_ERROR("Failed to call /set_pen");
	}
	
	stage = 1;
	return true;
}

bool almost_equal(const double d1, const double d2, const double epsilon=1.0e-2)
{
	return (fabs(d1-d2) < epsilon); 
}

const double PI=3.14159265358979323846;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "turtle_rect");
	ros::NodeHandle nh;
	
	const auto sub = nh.subscribe("turtle1/pose", 1000, callback);
	const auto pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	ros::ServiceServer start = nh.advertiseService("trect/start", start_method);
	
	double max_xdot;
    double max_wdot;
    double frequency;
    
    std_msgs::String msg;  //source: https://automaticaddison.com/how-to-create-a-publisher-node-in-ros-noetic/
    std::stringstream ss;
	
	ss   << '\n';
	if (nh.getParam("max_xdot", max_xdot))
	{
		ss << "max_xdot: " << max_xdot << '\n';
	}
	if (nh.getParam("max_wdot", max_wdot))
	{
		ss << "max_wdot: " << max_wdot << '\n';
	}
	if (nh.getParam("frequency", frequency))
	{
		ss << "frequency: " << frequency << '\n';
	}
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());

	ros::Rate r(frequency);
	
	while(ros::ok())
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0;
		cmd_vel.linear.y = 0;
		cmd_vel.linear.z = 0;
		cmd_vel.angular.x = 0;
		cmd_vel.angular.y = 0;
		cmd_vel.angular.z = 0;
		switch(stage)
		{
			case 1:
				cmd_vel.linear.x = max_xdot;
				cmd_vel.angular.z = 0;
				if (almost_equal(x, x_rect+width))
				{
					++stage;
				}
				break;
			case 2:
				cmd_vel.linear.x = 0;
				cmd_vel.angular.z = max_wdot;
				if (almost_equal(theta, PI/2))
				{
					++stage;
				}
				break;
			case 3:
				cmd_vel.linear.x = max_xdot;
				cmd_vel.angular.z = 0;
				if (almost_equal(y, y_rect+height))
				{
					++stage;
				}
				break;
			case 4:
				cmd_vel.linear.x = 0;
				cmd_vel.angular.z = max_wdot;
				if (almost_equal(theta, PI))
				{
					++stage;
				}
				break;
			case 5:
				cmd_vel.linear.x = max_xdot;
				cmd_vel.angular.z = 0;
				if (almost_equal(x, x_rect))
				{
					++stage;
				}
				break;
			case 6:
				cmd_vel.linear.x = 0;
				cmd_vel.angular.z = max_wdot;
				if (almost_equal(theta, -PI/2))
				{
					++stage;
				}
				break;
			case 7:
				cmd_vel.linear.x = max_xdot;
				cmd_vel.angular.z = 0;
				if (almost_equal(y, y_rect))
				{
					++stage;
				}
				break;
			case 8:
				cmd_vel.linear.x = 0;
				cmd_vel.angular.z = max_wdot;
				if (almost_equal(theta, 0.0))
				{
					++stage;
				}
				break;
			case 9:
				cmd_vel.linear.x = 0;
				cmd_vel.angular.z = 0;
				break;
			
		}    	
		pub.publish(cmd_vel);
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
	
