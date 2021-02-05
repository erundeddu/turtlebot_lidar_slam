#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief model the kinematics of a differential drive robot

#include "rigid2d/rigid2d.hpp"

namespace rigid2d
{	
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
		/// left wheel velocity
		double l_vel = 0.0;
		/// right wheel velocity
		double r_vel = 0.0;
	};
	
	/// \brief a way to track the pose of a diff drive robot through odometry
	class DiffDrive
	{
	private:
		RobotPose m_q; // pose (theta, x, y) of the diff drive robot
		double m_wheel_base;  // length of robot wheel base (m)
		double m_wheel_radius;  // wheel radius (m)
		double m_l_wheel_phi;  // angular displacement of left wheel (radians)
		double m_r_wheel_phi;  // angular displacement of right wheel (radians)
	public:
		/// \brief Track a diff drive robot, full initial configuration
		/// \param wheel_base - distance between wheels of the robot
		/// \param wheel_radius - radius of the robot wheels
		/// \param l_wheel_phi - initial angular displacement of left wheel
		/// \param r_wheel_phi - initial angular displacement of right wheel
		DiffDrive(double wheel_base, double wheel_radius, double l_wheel_phi, double r_wheel_phi);
		
		/// \brief Track a diff drive robot, full initial configuration with 0 initial angular displacement of wheels
		/// \param wheel_base - distance between wheels of the robot
		/// \param wheel_radius - radius of the robot wheels
		DiffDrive(double wheel_base, double wheel_radius);
		
		
		/// \brief Track a diff drive robot, uninitialized physical parameters
		DiffDrive();
		
		/// \brief sets value of wheel_base and wheel_radius private members
		/// \param wheel_base - distance between wheels of the robot
		/// \param wheel_radius - radius of the robot wheels
		void setPhysicalParams(double wheel_base, double wheel_radius);
		
		/// \brief update diff drive pose 
		/// \param l_wheel_phi_new - new angular displacement of left wheel of the robot
		/// \param r_wheel_phi_new - new angular displacement of right wheel of the robot
		void updatePose(double l_wheel_phi_new, double r_wheel_phi_new);
		
		/// \brief get latest body twist given change in wheel angle
		/// \param l_wheel_phi_new - new angular displacement of left wheel of the robot
		/// \param r_wheel_phi_new - new angular displacement of right wheel of the robot
		/// \param dt - time step between old and new configurations
		/// \return the current body twist
		Twist2D getBodyTwist(double l_wheel_phi_new, double r_wheel_phi_new, double dt) const;
		
		/// \brief converts a twist to wheel velocities
		/// \param tw - twist to be converted
		/// \return left and right wheel velocities
		WheelVel twist2WheelVel(const Twist2D & tw) const;
		
		/// \brief get the heading angle of the robot
		/// \return heading angle of the robot
		double getTheta() const;
		
		/// \brief get the x coordinate of the robot
		/// \return x coordinate of the robot
		double getX() const;
		
		/// \brief get the y coordinate of the robot
		/// \return y coordinate of the robot
		double getY() const;
				
		/// \brief get the left wheel angular displacement
		/// \return left wheel angular displacement
		double getLWheelPhi() const;
		
		/// \brief get the right wheel angular displacement
		/// \return right wheel angular displacement
		double getRWheelPhi() const;
	};
}		
#endif
