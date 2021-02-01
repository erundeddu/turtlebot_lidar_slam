#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief class to model the kinematics of a differential drive robot
//TODO doxygen documentation

#include "rigid2d/rigid2d.hpp"

namespace rigid2d
{
	class DiffDrive
	{
	private:
		RobotPose m_q; // pose (theta, x, y) of the diff drive robot
		double m_wheel_base;  // length of robot wheel base (m)
		double m_wheel_radius;  // wheel radius (m)
		double m_r_wheel_phi;  // angular displacement of right wheel (radians)
		double m_l_wheel_phi;  // angular displacement of left wheel (radians)
	public:
		DiffDrive(RobotPose q, double wheel_base, double wheel_radius, double r_wheel_phi, double l_wheel_phi);
		
		DiffDrive(RobotPose q, double wheel_base, double wheel_radius);
		
		DiffDrive(double wheel_base, double wheel_radius);
		
		void UpdatePose(r_wheel_phi_new, l_wheel_phi_new);
		
		WheelVel Twist2WheelVel(Twist2D & tw) const;
	};
	
	struct RobotPose
	{
		double theta = 0.0;
		double x = 0.0;
		double y = 0.0;
	};
	
	struct WheelVel
	{
		double r_vel = 0.0;
		double l_vel = 0.0;
	};
}		
#endif
