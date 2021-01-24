#include <iostream>
#include <string>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "trect/start.h"

class TurtleRect:
{
	public:
		TurtleRect():
			nh{},
			pub(nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);  // source: hhttp://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
			sub(nh.subscribe("turtle1/pose", 1000, &TurtleRect::callback, this)),
			start(nh.advertiseService("start", &TurtleRect::start_method, this));
		{
			read_param();
			timer = nh.createTimer(ros::Duration(1/m_frequency)), &TurtleRect::main_loop, this));
		}
		
		void callback(const turtlesim::Pose::ConstPtr& msg) const // source: https://www.fer.unizg.hr/_download/repository/lec07-ros-programming-cpp.pdf
		{
			m_x = msg->x;
			m_y = msg->y;
			m_theta = msg->theta;
		}
		
		void main_loop(const ros::TimerEvent &) const
		{
			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
			cmd_vel.linear.z = 0;
			cmd_vel.angular.x = 0;
			cmd_vel.angular.y = 0;
			cmd_vel.angular.z = 0;
			switch(m_stage)
			{
				case 1:
					cmd_vel.linear.x = m_max_xdot;
					cmd_vel.angular.z = 0;
					if (almost_equal(m_x, m_x_rect+m_width))
					{
						++m_stage;
					}
					break;
				case 2:
					cmd_vel.linear.x = 0;
					cmd_vel.angular.z = m_max_wdot;
					if (almost_equal(m_theta, PI/2))
					{
						++m_stage;
					}
					break;
				case 3:
					cmd_vel.linear.x = m_max_xdot;
					cmd_vel.angular.z = 0;
					if (almost_equal(m_y, m_y_rect+m_height))
					{
						++m_stage;
					}
					break;
				case 4:
					cmd_vel.linear.x = 0;
					cmd_vel.angular.z = m_max_wdot;
					if (almost_equal(m_theta, PI))
					{
						++m_stage;
					}
				case 5:
					cmd_vel.linear.x = m_max_xdot;
					cmd_vel.angular.z = 0;
					if (almost_equal(m_x, m_x_rect))
					{
						++m_stage;
					}
					break;
				case 6:
					cmd_vel.linear.x = 0;
					cmd_vel.angular.z = m_max_wdot;
					if (almost_equal(m_theta, 3*PI/2))
					{
						++m_stage;
					}
					break;
				case 7:
					cmd_vel.linear.x = m_max_xdot;
					cmd_vel.angular.z = 0;
					if (almost_equal(m_y, m_y_rect))
					{
						++m_stage;
					}
					break;
				case 8:
					cmd_vel.linear.x = 0;
					cmd_vel.angular.z = m_max_wdot;
					if (almost_equal(m_theta, 0.0))
					{
						++m_stage;
					}
					break;
			}
			pub.publish(cmd_vel);
		}
		
		void read_param()  // source: https://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters
		{
			if (nh.getParam("max_xdot", m_max_xdot))
			{
				ROS_INFO("max_xdot: %f\n", m_max_xdot);
			}
			if (nh.getParam("max_wdot", m_max_wdot))
			{
				ROS_INFO("max_wdot: %f\n", m_max_wdot);
			}
			if (nh.getParam("frequency", m_frequency))
			{
				ROS_INFO("frequency: %f\n", m_frequency);
			}
		}
		
		void start_method(trect::start::Request & req, trect::start::Response & res)
		{
			m_x_rect = req.x_rect;
			m_y_rect = req.y_rect;
			m_width = req.width;
			m_height = req.height;
			//TODO clear
			//TODO draw rectangle
			++m_stage;
		}
			
		
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::ServiceServer start;
		ros::Timer timer;
		double m_x;
		double m_y;
		double m_theta;
		double m_x_rect;
		double m_y_rect;
		double m_width;
		double m_height;
		double m_max_xdot;
		double m_max_wdot;
		double m_frequency;
		int m_stage = 0;
};

constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-2)
{
	return (fabs(d1-d2) < epsilon); 
}

constexpr double PI=3.14159265358979323846;

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "turtle_rect");
	TurtleRect trect;
	ros::spin();
	return 0;
}
