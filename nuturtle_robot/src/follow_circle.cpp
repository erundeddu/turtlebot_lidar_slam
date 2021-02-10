/// \file
/// \brief publishes commands that let the robot drive in a circle given inputs radius and speed
///
/// PARAMETERS:
///		~circle_radius (double): the radius of the circle along which the robot moves
///		~speed (double): the linear speed at which the robot moves along the circle
/// PUBLISHES:
///		cmd_vel (geometry_msgs/Twist): commanded twist
///	SERVICES:
///		control (nuturtle_robot/control): causes the robot to travel either clockwise, counter clockwise or stop

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include "nuturtle_robot/control.h"

/// ctrl = 1: clockwise
/// ctrl = 2: counterclockwise
/// ctrl = 3: stop
static int ctrl = 3;


/// \brief updates the state of the robot to follow clockwise circle, counterclockwise circle or stop
/// \param req - service request package of type nuturtle_robot::control::Request (containing commanded state of the circle to be followed)
/// \return true if all services were successfully called, else false
bool control_method(nuturtle_robot::control::Request & req, nuturtle_robot::control::Response &)
{
	ctrl = req.ctrl;
	return true;
}

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "follow_circle");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	double circle_radius = 0.15;
	double speed = 0.1;
	
	np.getParam("circle_radius", circle_radius);
	np.getParam("speed", speed);
	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	geometry_msgs::Twist tw;
	
	ros::ServiceServer srv = n.advertiseService("control", control_method);
	
	ros::Rate r(100);
	
	while(n.ok())
	{
		ros::spinOnce(); 
		switch (ctrl)
		{
			case 1:
				tw.linear.x = speed;
				tw.angular.z = -speed/circle_radius;
				break;
			case 2:
				tw.linear.x = speed;
				tw.angular.z = speed/circle_radius;
				break;
			case 3:
				tw.linear.x = 0.0;
				tw.angular.z = 0.0;
				break;
		}
		pub.publish(tw);
		r.sleep();
	}
	
	return 0;
}
	
