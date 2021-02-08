/// \file
/// \brief low-level control and sensor routines
///
/// PUBLISHES:
///     wheel_cmd (nuturtlebot/WheelCommands): makes the turtlebot follow the specified twist
///		joint_states (sensor_msgs/JointState): provides angle (rad) and velocity (rad/s) of the wheels based on encoder data
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): commanded twist
///		sensor_data (nuturtlebot/SensorData): wheel angles encoder data

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include <string>
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/set_pose.h"

static rigid2d::DiffDrive dd(160.0, 33.0);  // using the turtlebot3 burger wheel base and wheel radius
static bool started(false);
static bool publish_cmd_vel(false);
static bool publish_sensor_data(false);
static nuturtlebot::WheelCommands wheel_cmds;
static sensor_msgs::JointState js;


/// \brief converts an angular wheel speed to a command value in range [-256, 256]
/// \param ang_speed - the wheel angular speed in rad/s
/// \return a motor command value in range [-256, 256] proportional to the commanded angular wheel speed
int wheelSpeed2Command(double ang_speed)
{
	using namespace rigid2d;
	const double max_speed = 57.0*2.0*PI/60.0;
	int command = 256 * ang_speed/max_speed;
	if (command > 256)
	{
		command = 256;
	}
	if (command < -256)
	{
		command = -256;
	}
	return command;
}

/// \brief publishes commanded wheel velocities of the turtlebot given a twist
/// \param msg - a pointer to the geometry_msgs/Twist message with a 2D body twist commanded to the turtlebot
void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr & msg)
{
	using namespace rigid2d;
	Vector2D v(msg -> linear.x, msg -> linear.y);
	Twist2D tw(v, msg -> angular.z);
	WheelVel wv = dd.twist2WheelVel(tw);
	wheel_cmds.left_velocity = wheelSpeed2Command(wv.l_vel);
	wheel_cmds.right_velocity = wheelSpeed2Command(wv.r_vel);
	publish_cmd_vel = true;
}

/// \brief converts the turtlebot motor encoder value to a [-PI, PI] value 
/// \param encoder - the motor encoder counts
/// \return a motor command value in range [-256, 256] proportional to the commanded angular wheel speed
double encoder2rad(int encoder)
{
	using namespace rigid2d;
	const int max_count = 4096;
	double rad = normalize_angle(encoder*2.0*PI/max_count);
	return rad;
}

/// \brief publishes angular position and velocity of the robot wheels given encoder data
/// \param msg - a pointer to the nuturtlebot/SensorData message with motor encoder counts
void callback_sensor_data(const nuturtlebot::SensorData::ConstPtr & msg)
{
	using namespace rigid2d;
	static ros::Time current_time;
	static ros::Time last_time;
	current_time = ros::Time::now();
	
	js.name[0] = "left_wheel_joint";
	js.name[1] = "right_wheel_joint";
	
	static double left_rad;
	left_rad = encoder2rad(msg -> left_encoder);
	static double left_rad_prev;
	static double right_rad;
	right_rad = encoder2rad(msg -> right_encoder);
	static double right_rad_prev;
	
	dd.updatePose(left_rad, right_rad);
	js.position[0] = left_rad;
	js.position[1] = right_rad;
	
	if (!started)
	{
		js.velocity[0] = 0.0;
		js.velocity[1] = 0.0;
		started = true;
	}
	else
	{
		double dt = (current_time - last_time).toSec();
		js.velocity[0] = normalize_angular_difference(left_rad, left_rad_prev)/dt;
		js.velocity[1] = normalize_angular_difference(right_rad, right_rad_prev)/dt;
	}
	last_time = current_time;
	left_rad_prev = left_rad;
	right_rad_prev = right_rad;
	publish_sensor_data = true;
}	

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "turtle_interface");
	ros::NodeHandle n;
	ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 1000, callback_cmd_vel);
	ros::Subscriber sub_sensor_data = n.subscribe("sensor_data", 1000, callback_sensor_data);	
	ros::Publisher pub_cmd_vel = n.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 1000);
	ros::Publisher pub_sensor_data = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	
	ros::Rate r(100);
	
	while(n.ok())
	{
		ros::spinOnce(); 
		r.sleep();
		if (publish_cmd_vel)
		{
			pub_cmd_vel.publish(wheel_cmds);
			publish_cmd_vel = false;
		}
		if (publish_sensor_data)
		{
			pub_sensor_data.publish(js);
			publish_sensor_data = false;
		}
	}
	
	return 0;
}
