#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>

static int called_trans = false;
static int called_rot = false;

void callback_trans(const nuturtlebot::WheelCommands::ConstPtr & msg)
{
	if (((msg -> left_velocity !=0) && (msg -> right_velocity != 0)) || called_trans)
	{
		called_trans = true;
		CHECK(msg -> left_velocity == 42);
		CHECK(msg -> right_velocity == 42);
	}
	else
	{
		CHECK(msg -> left_velocity == 0);
		CHECK(msg -> right_velocity == 0);
	}
}

void callback_rot(const nuturtlebot::WheelCommands::ConstPtr & msg)
{
	if (((msg -> left_velocity !=0) && (msg -> right_velocity != 0)) || called_rot)
	{
		called_rot = true;
		CHECK(msg -> left_velocity == -207);
		CHECK(msg -> right_velocity == 207);
	}
	else
	{
		CHECK(msg -> left_velocity == 0);
		CHECK(msg -> right_velocity == 0);
	}
}

TEST_CASE("pure translation cmd_vel", "[cmd_vel]")
{
	ros::NodeHandle nh;
	const auto sub = nh.subscribe("wheel_cmd", 1000, callback_trans);
	const auto pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000, true);
	geometry_msgs::Twist tw;
	tw.linear.x = 33.0;
	tw.linear.y = 0.0;
	tw.angular.z = 0.0;
	pub.publish(tw);
	ros::Rate r(100.0);
	for(int i = 0; ros::ok() && i != 200; ++i)
	{
		ros::spinOnce();
		r.sleep();
	}
	CHECK(called_trans);
}

TEST_CASE("pure rotation cmd_vel", "[cmd_vel]")
{
	ros::NodeHandle nh;
	const auto sub = nh.subscribe("wheel_cmd", 1000, callback_rot);
	const auto pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000, true);
	geometry_msgs::Twist tw;
	tw.linear.x = 0.0;
	tw.linear.y = 0.0;
	tw.angular.z = 2.0;
	pub.publish(tw);
	ros::Rate r(100.0);
	for(int i = 0; ros::ok() && i != 200; ++i)
	{
		ros::spinOnce();
		r.sleep();
	}
	CHECK(called_rot);
}
