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

static rigid2d::DiffDrive dd(160.0, 33.0)  // using the turtlebot3 burger wheel base and wheel radius
static bool started(false);

int wheelSpeed2Command(double ang_speed)
{
	const double max_speed = 57.0*2.0*PI/60.0;
	int command = ang_speed/max_speed;
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

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr & msg)
{
	using namespace rigid2d;
	Vector2D v(msg -> linear.x, msg -> linear.y);
	Twist2D tw(v, msg -> angular.z);
	WheelVel wv = dd.twist2WheelVel(tw);
	
	static ros::Publisher pub_cmd_vel = n.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 50);
	nuturtlebot::WheelCommands wheel_cmds;
	wheel_cmds.left_velocity = wheelSpeed2Command(wv.l_vel);
	wheel_cmds.right_velocity = wheelSpeed2Command(wv.r_vel);
	pub_cmd_vel.publish(wheel_cmds);
}

double encoder2rad(int encoder)
	const int max_count = 4096;
	double rad = encoder*2.0*PI/max_count;
	return rad;
}

void callback_sensor_data(const nuturtlebot::SensorData::ConstPtr & msg)
{
/*
	using namespace rigid2d;
	static ros::Time current_time;
	static ros::Time last_time;
	current_time = ros::Time::now();
	
	int left_encoder = msg -> left_encoder;
	int right_encoder = msg -> right_encoder;
	
	sensor_msgs::JointState js;
	js.name[0] = "left_wheel_joint";
	js.name[1] = "right_wheel_joint";
	static double left_rad_0 = encoder2rad(left_encoder);
	static double right_rad_0 = encoder2rad(right_encoder);
	
	left_rad = encoder2rad(left_encoder);
	right_rad = encoder2rad(right_encoder);
	
	if (!started)
	{
		dd.updatePose(left_rad, right_rad);
		js.position[0] = left_rad;
		js.position[1] = right_rad;
		js.velocity[0] = 0.0;
		js.velocity[1] = 0.0;
		started = true;
	}
	else
	{
		
	}
	
	js.position[0] = left_rad;
	js.position[1] = right_rad;
	js.velocity[0] = //TODO depending on dt
	js.velocity[1] = //TODO depending on dt
	
	static ros::Publisher pub_sensor_data = n.advertise<sensor_msgs::JointState>("joint_states", 50);
	pub_sensor_data.publish(js);
	last_time = current_time;
	*/
}	

int main(int argc, char** argv)
{
	using namespace rigid2d;
	
	ros::init(argc, argv, "turtle_interface");
	ros::NodeHandle n;
	ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 50, callback_cmd_vel);
	ros::Subscriber sub_sensor_data = n.subscribe("sensor_data", 50, callback_sensor_data);	
	
	ros::Rate r(100);
	
	while(n.ok())
	{
		ros::spinOnce(); 
		r.sleep();
	}
	
	return 0;
}
