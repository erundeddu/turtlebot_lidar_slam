#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief model the kinematics of a differential drive robot

#include "rigid2d/rigid2d.hpp"

namespace rigid2d
{
	/// \brief a way to track the pose of a diff drive robot through odometry
	class DiffDrive
	{
	private:
		RobotPose m_q; // pose (theta, x, y) of the diff drive robot
		double m_wheel_base;  // length of robot wheel base (m)
		double m_wheel_radius;  // wheel radius (m)
		double m_r_wheel_phi;  // angular displacement of right wheel (radians)
		double m_l_wheel_phi;  // angular displacement of left wheel (radians)
	public:
		/// \brief Track a diff drive robot, full initial configuration
		/// \param q - initial robot pose (theta, x, y)
		/// \param wheel_base - distance between wheels of the robot
		/// \param wheel_radius - radius of the robot wheels
		/// \param r_wheel_phi - initial angular displacement of left wheel
		/// \param l_wheel_phi - initial angular displacement of right wheel
		DiffDrive(RobotPose q, double wheel_base, double wheel_radius, double r_wheel_phi, double l_wheel_phi);
		
		/// \brief Track a diff drive robot, full initial configuration with 0 initial angular displacement of wheels
		/// \param q - initial robot pose (theta, x, y)
		/// \param wheel_base - distance between wheels of the robot
		/// \param wheel_radius - radius of the robot wheels
		DiffDrive(RobotPose q, double wheel_base, double wheel_radius);
		
		/// \brief Track a diff drive robot, full initial configuration with 0 initial angular displacement of wheels and pose
		/// \param wheel_base - distance between wheels of the robot
		/// \param wheel_radius - radius of the robot wheels
		DiffDrive(double wheel_base, double wheel_radius);
		
		/// \brief update diff drive pose 
		/// \param r_wheel_phi_new - new angular displacement of right wheel of the robot
		/// \param l_wheel_phi_new - new angular displacement of left wheel of the robot
		void UpdatePose(double r_wheel_phi_new, double l_wheel_phi_new);
		
		/// \brief converts a twist to wheel velocities
		/// \param tw - twist to be converted
		/// \return left and right wheel velocities
		WheelVel Twist2WheelVel(Twist2D & tw) const;
	};
	
	/// \brief 2D pose of a diff drive robot
	struct RobotPose
	{
		/// heading angle of robot
		double theta = 0.0;
		/// x coordinate of robot
		double x = 0.0;
		/// y coordinate of robot
		double y = 0.0;
	};
	
	/// \brief set of wheel velocities for a diff drive robot
	struct WheelVel
	{
		/// right wheel velocity
		double r_vel = 0.0;
		/// left wheel velocity
		double l_vel = 0.0;
	};
}		
#endif
