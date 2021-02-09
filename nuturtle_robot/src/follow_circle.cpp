/// \file
/// \brief publishes commands that let the robot drive in a circle given inputs radius and speed
///
/// PUBLISHES:

/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): commanded twist
///		sensor_data (nuturtlebot/SensorData): wheel angles encoder data
///	SERVICES:

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include <string>
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/set_pose.h"

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
